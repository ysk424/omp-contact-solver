"""Blender UI and bake pipeline for the OpenMP cloth DLL."""

import math
import time

import bpy
from bpy.props import (
    BoolProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Operator, Panel, PropertyGroup

from .native import NativeSolverError, Vec3, get_library


_BAKE_TAG = "omp_contact_solver_bake_version"
_PREPARED_COLLECTION_TAG = "omp_contact_solver_prepared_collection_version"
_PREPARED_OBJECT_TAG = "omp_contact_solver_prepared_object_version"
_PREPARED_ROLE_TAG = "omp_contact_solver_role"
_PREPARED_SOURCE_TAG = "omp_contact_solver_source"
_PREPARED_SEAMS_TAG = "omp_contact_solver_seam_pairs"
_PREPARED_SEAM_DISTANCE_TAG = "omp_contact_solver_seam_distance"
_PREPARED_SEAM_ENABLED_TAG = "omp_contact_solver_seam_enabled"
_PREPARED_CROP_ENABLED_TAG = "omp_contact_solver_static_crop_enabled"
_PREPARED_CROP_MIN_TAG = "omp_contact_solver_static_crop_min_z"
_PREPARED_CROP_MAX_TAG = "omp_contact_solver_static_crop_max_z"
_PREPARED_COLLECTION_NAME = "OMP Contact Simulation"
_PREPARED_VERSION = 3
_STATIC_TWICE_AREA_FILTER = 1.25e-7
_STATIC_CROP_GROUP = "OMP_STATIC_CROP"
_STATIC_CROP_MODIFIER = "OMP Static Crop"
_BAKE_RUNNING = False


def _mesh_object(_self, obj) -> bool:
    return obj is not None and obj.type == "MESH"


def _triangulated_mesh(mesh, matrix_world):
    mesh.calc_loop_triangles()
    vertices = [tuple(matrix_world @ vertex.co) for vertex in mesh.vertices]
    triangles = [tuple(triangle.vertices) for triangle in mesh.loop_triangles]
    return vertices, triangles


def _shell_mesh(obj):
    return _triangulated_mesh(obj.data, obj.matrix_world)


def _static_topology_signature(mesh):
    return (
        len(mesh.vertices),
        len(mesh.edges),
        len(mesh.polygons),
        len(mesh.loops),
    )


def _filtered_static_mesh(obj, depsgraph):
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    if mesh is None:
        raise RuntimeError("STATIC object could not be evaluated as a mesh")
    try:
        vertices, triangles = _triangulated_mesh(mesh, evaluated.matrix_world)
        accepted = []
        skipped = 0
        for i0, i1, i2 in triangles:
            a = vertices[i0]
            b = vertices[i1]
            c = vertices[i2]
            ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            cross = (
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            )
            twice_area = math.sqrt(sum(value * value for value in cross))
            if (
                math.isfinite(twice_area)
                and twice_area > _STATIC_TWICE_AREA_FILTER
            ):
                accepted.append((i0, i1, i2))
            else:
                skipped += 1
        return vertices, accepted, skipped, _static_topology_signature(mesh)
    finally:
        evaluated.to_mesh_clear()


def _evaluated_static_vertices(obj, depsgraph):
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    if mesh is None:
        raise RuntimeError("Animated STATIC could not be evaluated as a mesh")
    try:
        vertices = [
            tuple(evaluated.matrix_world @ vertex.co) for vertex in mesh.vertices
        ]
        return vertices, _static_topology_signature(mesh)
    finally:
        evaluated.to_mesh_clear()


def _evaluated_snapshot(source, depsgraph, name):
    evaluated = source.evaluated_get(depsgraph)
    mesh = bpy.data.meshes.new_from_object(
        evaluated,
        preserve_all_data_layers=True,
        depsgraph=depsgraph,
    )
    if mesh is None:
        raise RuntimeError(f"{source.name} could not be evaluated as a mesh")
    try:
        mesh.name = f"{name}_Mesh"
        mesh.transform(evaluated.matrix_world)
        mesh.update()
        obj = bpy.data.objects.new(name, mesh)
    except Exception:
        bpy.data.meshes.remove(mesh)
        raise
    obj.color = tuple(source.color)
    obj.show_in_front = source.show_in_front
    return obj


def _animated_static_copy(source, name):
    """Copy the object stack while sharing untouched source mesh data."""
    obj = source.copy()
    obj.name = name
    obj.hide_viewport = False
    obj.hide_render = True
    obj.display_type = "WIRE"
    obj.show_in_front = True
    return obj


def _configure_static_crop(obj, depsgraph, minimum_z, maximum_z):
    """Add a topology-stable final Mask selected in frame-one world space."""
    if not minimum_z < maximum_z:
        raise RuntimeError("STATIC crop minimum must be below its maximum")
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    if mesh is None:
        raise RuntimeError("STATIC could not be evaluated for cropping")
    try:
        if len(mesh.vertices) != len(obj.data.vertices):
            raise RuntimeError(
                "STATIC crop requires deformation-only source modifiers"
            )
        world_z = [
            float((evaluated.matrix_world @ vertex.co).z)
            for vertex in mesh.vertices
        ]
        selected = set()
        kept_polygons = 0
        for polygon in mesh.polygons:
            heights = [world_z[index] for index in polygon.vertices]
            if min(heights) <= maximum_z and max(heights) >= minimum_z:
                selected.update(int(index) for index in polygon.vertices)
                kept_polygons += 1
        polygon_count = len(mesh.polygons)
    finally:
        evaluated.to_mesh_clear()
    if not selected or kept_polygons == 0:
        raise RuntimeError("STATIC crop box contains no body polygons")

    # Vertex-group weights live in mesh custom data. Isolate the prepared copy
    # so adding the crop never touches the user's source body.
    obj.data = obj.data.copy()
    group = obj.vertex_groups.get(_STATIC_CROP_GROUP)
    if group is None:
        group = obj.vertex_groups.new(name=_STATIC_CROP_GROUP)
    group.add(sorted(selected), 1.0, "REPLACE")
    modifier = obj.modifiers.new(name=_STATIC_CROP_MODIFIER, type="MASK")
    modifier.vertex_group = group.name
    modifier.invert_vertex_group = False
    return len(selected), len(obj.data.vertices), kept_polygons, polygon_count


def _is_prepared_object(obj) -> bool:
    return bool(obj is not None and obj.get(_PREPARED_OBJECT_TAG, 0))


def _is_prepared_collection(collection) -> bool:
    return bool(collection is not None and collection.get(_PREPARED_COLLECTION_TAG, 0))


def _validate_shell_mesh(mesh) -> None:
    mesh.calc_loop_triangles()
    used_vertices = set()
    rejected = 0
    for triangle in mesh.loop_triangles:
        i0, i1, i2 = triangle.vertices
        used_vertices.update((i0, i1, i2))
        a = mesh.vertices[i0].co
        b = mesh.vertices[i1].co
        c = mesh.vertices[i2].co
        twice_area = (b - a).cross(c - a).length
        if not math.isfinite(twice_area) or twice_area <= 1.0e-7:
            rejected += 1
    if rejected:
        raise RuntimeError(
            f"Prepared SHELL contains {rejected} triangles that are too small"
        )
    orphan_count = len(mesh.vertices) - len(used_vertices)
    if orphan_count:
        raise RuntimeError(
            f"Prepared SHELL contains {orphan_count} vertices outside its faces"
        )


def _detect_seam_pairs(mesh, max_distance: float) -> list[tuple[int, int]]:
    """Greedily pair nearby boundary vertices from disconnected components."""
    if not (max_distance > 0.0):
        return []

    coordinates = [vertex.co.copy() for vertex in mesh.vertices]
    adjacency = [set() for _vertex in coordinates]
    edge_counts = {}
    for polygon in mesh.polygons:
        vertices = list(polygon.vertices)
        for index, a in enumerate(vertices):
            b = vertices[(index + 1) % len(vertices)]
            edge = (a, b) if a < b else (b, a)
            edge_counts[edge] = edge_counts.get(edge, 0) + 1
            adjacency[a].add(b)
            adjacency[b].add(a)

    components = [-1] * len(coordinates)
    component = 0
    for root in range(len(coordinates)):
        if components[root] >= 0:
            continue
        components[root] = component
        stack = [root]
        while stack:
            vertex = stack.pop()
            for neighbor in adjacency[vertex]:
                if components[neighbor] < 0:
                    components[neighbor] = component
                    stack.append(neighbor)
        component += 1

    boundary = sorted(
        {vertex for edge, count in edge_counts.items() if count == 1 for vertex in edge}
    )
    cell_size = max_distance
    grid = {}
    for vertex in boundary:
        point = coordinates[vertex]
        cell = tuple(math.floor(float(point[axis]) / cell_size) for axis in range(3))
        grid.setdefault(cell, []).append(vertex)

    maximum_squared = max_distance * max_distance
    candidates = []
    for a in boundary:
        point = coordinates[a]
        cell = tuple(math.floor(float(point[axis]) / cell_size) for axis in range(3))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    neighbor_cell = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                    for b in grid.get(neighbor_cell, ()):
                        if b <= a or components[a] == components[b]:
                            continue
                        distance_squared = (coordinates[a] - coordinates[b]).length_squared
                        if distance_squared <= maximum_squared:
                            candidates.append((distance_squared, a, b))

    candidates.sort()
    paired = set()
    seams = []
    for _distance_squared, a, b in candidates:
        if a in paired or b in paired:
            continue
        paired.add(a)
        paired.add(b)
        seams.append((a, b))
    return seams


def _prepared_seam_pairs(shell) -> list[tuple[int, int]]:
    flattened = list(shell.get(_PREPARED_SEAMS_TAG, ()))
    if len(flattened) % 2:
        raise RuntimeError("Prepared SHELL seam data is invalid; run Prepare again")
    return [
        (int(flattened[index]), int(flattened[index + 1]))
        for index in range(0, len(flattened), 2)
    ]


def _restore_source_shell_visibility(settings) -> None:
    if not settings.source_shell_hidden_by_prepare:
        return
    source = bpy.data.objects.get(settings.prepared_source_shell_name)
    if source is None:
        source = settings.shell_object
    if source is not None:
        source.hide_set(False)
    settings.source_shell_hidden_by_prepare = False


def _remove_prepared(settings, *, restore_visibility: bool) -> bool:
    if restore_visibility:
        _restore_source_shell_visibility(settings)

    collection = settings.prepared_collection
    objects = []
    for obj in (settings.prepared_shell_object, settings.prepared_static_object):
        if _is_prepared_object(obj):
            objects.append(obj)
    if _is_prepared_collection(collection):
        for obj in collection.objects:
            if _is_prepared_object(obj) and obj not in objects:
                objects.append(obj)

    settings.prepared_shell_object = None
    settings.prepared_static_object = None
    settings.prepared_collection = None
    settings.prepared_source_shell_name = ""
    settings.prepared_source_static_name = ""

    removed = bool(objects)
    for obj in objects:
        mesh = obj.data if obj.type == "MESH" else None
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh is not None and mesh.users == 0:
            bpy.data.meshes.remove(mesh)

    if (
        _is_prepared_collection(collection)
        and not collection.objects
        and not collection.children
    ):
        bpy.data.collections.remove(collection, do_unlink=True)
        removed = True
    return removed


def _prepared_pair(settings):
    shell = settings.prepared_shell_object
    static = settings.prepared_static_object
    if shell is None or static is None:
        raise RuntimeError("Run Prepare Simulation Copies first")
    if (
        shell.get(_PREPARED_OBJECT_TAG) != _PREPARED_VERSION
        or shell.get(_PREPARED_ROLE_TAG) != "SHELL"
        or static.get(_PREPARED_OBJECT_TAG) != _PREPARED_VERSION
        or static.get(_PREPARED_ROLE_TAG) != "STATIC"
    ):
        raise RuntimeError("Prepared simulation objects are invalid; run Prepare again")
    if (
        settings.shell_object is None
        or settings.static_object is None
        or shell.get(_PREPARED_SOURCE_TAG) != settings.shell_object.name
        or static.get(_PREPARED_SOURCE_TAG) != settings.static_object.name
    ):
        raise RuntimeError("Source objects changed; run Prepare again")
    if (
        bool(shell.get(_PREPARED_SEAM_ENABLED_TAG, False))
        != settings.seam_enabled
    ):
        raise RuntimeError("Seam detection setting changed; run Prepare again")
    if settings.seam_enabled and not math.isclose(
        float(shell.get(_PREPARED_SEAM_DISTANCE_TAG, -1.0)),
        settings.seam_search_distance,
        rel_tol=1.0e-6,
        abs_tol=1.0e-9,
    ):
        raise RuntimeError("Seam Distance changed; run Prepare again")
    if bool(static.get(_PREPARED_CROP_ENABLED_TAG, False)) != settings.static_crop_enabled:
        raise RuntimeError("STATIC crop setting changed; run Prepare again")
    if settings.static_crop_enabled and (
        not math.isclose(
            float(static.get(_PREPARED_CROP_MIN_TAG, math.nan)),
            settings.static_crop_min_z,
            rel_tol=1.0e-6,
            abs_tol=1.0e-9,
        )
        or not math.isclose(
            float(static.get(_PREPARED_CROP_MAX_TAG, math.nan)),
            settings.static_crop_max_z,
            rel_tol=1.0e-6,
            abs_tol=1.0e-9,
        )
    ):
        raise RuntimeError("STATIC crop range changed; run Prepare again")
    return shell, static


def _owned_bake(obj) -> bool:
    keys = obj.data.shape_keys
    return bool(keys and keys.get(_BAKE_TAG) == 1)


def _clear_owned_bake(obj) -> bool:
    if not obj.data.shape_keys:
        return False
    if not _owned_bake(obj):
        raise RuntimeError("SHELL has Shape Keys that are not owned by OMP Contact Solver")
    obj.shape_key_clear()
    return True


def _world_to_local_flat(matrix, positions):
    rows = [[float(matrix[row][column]) for column in range(4)] for row in range(3)]
    flattened = [0.0] * (len(positions) * 3)
    for index, (x, y, z) in enumerate(positions):
        offset = index * 3
        flattened[offset] = rows[0][0] * x + rows[0][1] * y + rows[0][2] * z + rows[0][3]
        flattened[offset + 1] = (
            rows[1][0] * x + rows[1][1] * y + rows[1][2] * z + rows[1][3]
        )
        flattened[offset + 2] = (
            rows[2][0] * x + rows[2][1] * y + rows[2][2] * z + rows[2][3]
        )
    return flattened


def _set_linear_interpolation(action) -> None:
    if hasattr(action, "fcurves"):
        curves = action.fcurves
    else:
        curves = (
            curve
            for layer in action.layers
            for strip in layer.strips
            if hasattr(strip, "channelbags")
            for channelbag in strip.channelbags
            for curve in channelbag.fcurves
        )
    for curve in curves:
        if curve.data_path == "eval_time":
            for point in curve.keyframe_points:
                point.interpolation = "LINEAR"


def _configure_absolute_shape_keys(shell, frame_start, frame_end) -> None:
    keys = shell.data.shape_keys
    keys.use_relative = False
    keys.eval_time = 0.0
    keys.keyframe_insert(data_path="eval_time", frame=frame_start)
    keys.eval_time = float((frame_end - frame_start) * 10)
    keys.keyframe_insert(data_path="eval_time", frame=frame_end)
    action = keys.animation_data.action if keys.animation_data else None
    if action is not None:
        _set_linear_interpolation(action)


class OCS_Settings(PropertyGroup):
    shell_object: PointerProperty(
        name="Source SHELL",
        description="Source garment evaluated at the first bake frame",
        type=bpy.types.Object,
        poll=_mesh_object,
    )
    static_object: PointerProperty(
        name="Source STATIC",
        description="Source collision body whose animation modifiers are copied and evaluated every frame",
        type=bpy.types.Object,
        poll=_mesh_object,
    )
    static_crop_enabled: BoolProperty(
        name="Crop STATIC Body",
        description="Keep a topology-stable animated collision band between two world-Z planes",
        default=True,
    )
    static_crop_min_z: FloatProperty(
        name="Lower Z",
        description="Lower world-Z cut plane; polygons crossing the plane are retained",
        default=0.40,
        subtype="DISTANCE",
        unit="LENGTH",
    )
    static_crop_max_z: FloatProperty(
        name="Upper Z",
        description="Upper world-Z cut plane; polygons crossing the plane are retained",
        default=1.45,
        subtype="DISTANCE",
        unit="LENGTH",
    )
    prepared_shell_object: PointerProperty(
        name="Prepared SHELL",
        type=bpy.types.Object,
        poll=_mesh_object,
        options={"HIDDEN"},
    )
    prepared_static_object: PointerProperty(
        name="Prepared STATIC",
        type=bpy.types.Object,
        poll=_mesh_object,
        options={"HIDDEN"},
    )
    prepared_collection: PointerProperty(
        name="Prepared Collection",
        type=bpy.types.Collection,
        options={"HIDDEN"},
    )
    prepared_source_shell_name: StringProperty(options={"HIDDEN"})
    prepared_source_static_name: StringProperty(options={"HIDDEN"})
    source_shell_hidden_by_prepare: BoolProperty(default=False, options={"HIDDEN"})
    frame_start: IntProperty(name="Start", default=1, min=-1048574, max=1048574)
    frame_end: IntProperty(name="End", default=120, min=-1048574, max=1048574)
    time_scale: FloatProperty(name="Time Scale", default=1.0, min=0.001, max=100.0)
    gravity: FloatVectorProperty(
        name="Gravity",
        default=(0.0, 0.0, -9.81),
        size=3,
        subtype="XYZ",
    )
    substeps: IntProperty(name="Substeps", default=4, min=1, max=128)
    pd_iterations: IntProperty(name="PD Iterations", default=8, min=1, max=256)
    pcg_iterations: IntProperty(name="PCG Iterations", default=64, min=1, max=4096)
    pcg_tolerance: FloatProperty(
        name="PCG Tolerance",
        default=1.0e-5,
        min=1.0e-8,
        max=0.1,
        precision=6,
    )
    collision_iterations: IntProperty(
        name="Collision Safety Passes",
        description="Hard post-solve contact projections; zero preserves coupled strain constraints",
        default=0,
        min=0,
        max=64,
    )
    velocity_damping: FloatProperty(
        name="Velocity Damping", default=0.01, min=0.0, max=1.0
    )
    thread_count: IntProperty(
        name="Threads",
        description="Zero uses the OpenMP maximum",
        default=0,
        min=0,
        max=1024,
    )
    density: FloatProperty(name="Density", default=1.0, min=1.0e-6, max=1.0e6)
    stretch_stiffness: FloatProperty(
        name="Stretch", default=5000.0, min=0.0, max=1.0e9
    )
    bend_stiffness: FloatProperty(name="Bend", default=5.0, min=0.0, max=1.0e9)
    strain_limit_enabled: BoolProperty(
        name="Strain Limit",
        description="Couple a per-triangle principal-stretch bound into the PD/ADMM system",
        default=True,
    )
    strain_limit_percent: FloatProperty(
        name="Maximum Stretch",
        description="Maximum projected tensile principal strain in percent",
        default=5.0,
        min=0.1,
        max=100.0,
        subtype="PERCENTAGE",
        precision=2,
    )
    strain_limit_stiffness: FloatProperty(
        name="Limit Solver Weight",
        description="ADMM penalty weight controlling strain-limit convergence",
        default=100000.0,
        min=1.0,
        max=1.0e9,
    )
    seam_enabled: BoolProperty(
        name="Auto Seam Threads",
        description="Connect nearby boundary vertices from disconnected SHELL parts",
        default=True,
    )
    seam_search_distance: FloatProperty(
        name="Seam Distance",
        description="Maximum rest distance for automatic seam pairing; run Prepare after changing",
        default=0.01,
        min=1.0e-6,
        max=1.0,
        subtype="DISTANCE",
        precision=4,
    )
    seam_stiffness: FloatProperty(
        name="Seam Stiffness",
        description="Finite seam strength solved together with cloth and contact constraints",
        default=100000.0,
        min=1.0,
        max=1.0e9,
    )
    thickness: FloatProperty(
        name="Thickness", default=0.002, min=0.0, max=1000.0, subtype="DISTANCE"
    )
    friction: FloatProperty(name="Friction", default=0.3, min=0.0, max=1.0)
    restitution: FloatProperty(name="Restitution", default=0.0, min=0.0, max=1.0)
    last_prepare_status: StringProperty(name="Prepare Status", default="Not prepared")
    last_prepare_skipped: IntProperty(name="Skipped STATIC Triangles", default=0)
    last_static_crop_vertices: StringProperty(default="-", options={"HIDDEN"})
    last_static_crop_polygons: StringProperty(default="-", options={"HIDDEN"})
    last_seam_count: IntProperty(name="Detected Seams", default=0)
    last_status: StringProperty(name="Status", default="Not baked")
    last_contacts: StringProperty(name="Contacts", default="-")
    last_residual: StringProperty(name="PCG Residual", default="-")
    last_strain: StringProperty(name="Maximum Principal Stretch", default="-")


class OCS_OT_set_active_shell(Operator):
    bl_idname = "ocs.set_active_shell"
    bl_label = "Use Active as SHELL"
    bl_description = "Assign the active mesh as the deformable SHELL"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == "MESH"

    def execute(self, context):
        context.scene.ocs_settings.shell_object = context.active_object
        return {"FINISHED"}


class OCS_OT_set_active_static(Operator):
    bl_idname = "ocs.set_active_static"
    bl_label = "Use Active as STATIC"
    bl_description = "Assign the active mesh as the animated STATIC collider"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == "MESH"

    def execute(self, context):
        context.scene.ocs_settings.static_object = context.active_object
        return {"FINISHED"}


class OCS_OT_prepare(Operator):
    bl_idname = "ocs.prepare"
    bl_label = "Prepare Simulation Copies"
    bl_description = (
        "Create simulation copies and preserve the STATIC animation modifier stack"
    )

    def execute(self, context):
        if _BAKE_RUNNING:
            self.report({"ERROR"}, "A solver bake is already running")
            return {"CANCELLED"}
        if context.mode != "OBJECT":
            self.report({"ERROR"}, "Switch Blender to Object Mode before preparing")
            return {"CANCELLED"}

        settings = context.scene.ocs_settings
        source_shell = settings.shell_object
        source_static = settings.static_object
        if source_shell is None or source_static is None:
            self.report({"ERROR"}, "Assign both source SHELL and source STATIC meshes")
            return {"CANCELLED"}
        if source_shell == source_static:
            self.report({"ERROR"}, "Source SHELL and STATIC must be different objects")
            return {"CANCELLED"}

        old_frame = context.scene.frame_current
        collection = None
        prepared_shell = None
        prepared_static = None
        try:
            context.scene.frame_set(settings.frame_start)
            _remove_prepared(settings, restore_visibility=True)
            settings.last_prepare_skipped = 0
            settings.last_seam_count = 0
            settings.last_static_crop_vertices = "-"
            settings.last_static_crop_polygons = "-"

            collection = bpy.data.collections.new(_PREPARED_COLLECTION_NAME)
            collection[_PREPARED_COLLECTION_TAG] = _PREPARED_VERSION
            context.scene.collection.children.link(collection)
            depsgraph = context.evaluated_depsgraph_get()

            prepared_shell = _evaluated_snapshot(
                source_shell,
                depsgraph,
                f"{source_shell.name}_OMP_SHELL",
            )
            collection.objects.link(prepared_shell)
            prepared_shell[_PREPARED_OBJECT_TAG] = _PREPARED_VERSION
            prepared_shell[_PREPARED_ROLE_TAG] = "SHELL"
            prepared_shell[_PREPARED_SOURCE_TAG] = source_shell.name
            _validate_shell_mesh(prepared_shell.data)
            seam_pairs = (
                _detect_seam_pairs(
                    prepared_shell.data,
                    settings.seam_search_distance,
                )
                if settings.seam_enabled
                else []
            )
            flattened_seams = [vertex for pair in seam_pairs for vertex in pair]
            if flattened_seams:
                prepared_shell[_PREPARED_SEAMS_TAG] = flattened_seams
            prepared_shell[_PREPARED_SEAM_DISTANCE_TAG] = settings.seam_search_distance
            prepared_shell[_PREPARED_SEAM_ENABLED_TAG] = settings.seam_enabled

            prepared_static = _animated_static_copy(
                source_static, f"{source_static.name}_OMP_STATIC"
            )
            collection.objects.link(prepared_static)
            prepared_static.hide_set(False)
            prepared_static[_PREPARED_OBJECT_TAG] = _PREPARED_VERSION
            prepared_static[_PREPARED_ROLE_TAG] = "STATIC"
            prepared_static[_PREPARED_SOURCE_TAG] = source_static.name
            prepared_static[_PREPARED_CROP_ENABLED_TAG] = settings.static_crop_enabled
            prepared_static[_PREPARED_CROP_MIN_TAG] = settings.static_crop_min_z
            prepared_static[_PREPARED_CROP_MAX_TAG] = settings.static_crop_max_z
            context.view_layer.update()
            depsgraph = context.evaluated_depsgraph_get()
            if settings.static_crop_enabled:
                kept_vertices, source_vertices, kept_polygons, source_polygons = (
                    _configure_static_crop(
                        prepared_static,
                        depsgraph,
                        settings.static_crop_min_z,
                        settings.static_crop_max_z,
                    )
                )
                settings.last_static_crop_vertices = (
                    f"{kept_vertices:,} / {source_vertices:,}"
                )
                settings.last_static_crop_polygons = (
                    f"{kept_polygons:,} / {source_polygons:,}"
                )
                context.view_layer.update()
                depsgraph = context.evaluated_depsgraph_get()
            (_static_vertices, static_triangles, skipped,
             _static_topology) = _filtered_static_mesh(prepared_static, depsgraph)
            if not static_triangles:
                raise RuntimeError("Prepared STATIC has no usable triangles")

            settings.prepared_collection = collection
            settings.prepared_shell_object = prepared_shell
            settings.prepared_static_object = prepared_static
            settings.prepared_source_shell_name = source_shell.name
            settings.prepared_source_static_name = source_static.name
            settings.last_prepare_skipped = skipped
            settings.last_seam_count = len(seam_pairs)
            settings.last_prepare_status = (
                f"Prepared animated STATIC in {collection.name}; "
                f"{len(seam_pairs)} seams; "
                f"{len(static_triangles):,} STATIC triangles; "
                f"skipped {skipped} tiny STATIC triangles"
            )
            settings.last_status = "Ready to bake"
            settings.last_contacts = "-"
            settings.last_residual = "-"
            settings.last_strain = "-"

            if not source_shell.hide_get():
                source_shell.hide_set(True)
                settings.source_shell_hidden_by_prepare = True
            for obj in context.selected_objects:
                obj.select_set(False)
            prepared_shell.select_set(True)
            context.view_layer.objects.active = prepared_shell
            self.report({"INFO"}, settings.last_prepare_status)
            return {"FINISHED"}
        except Exception as exc:
            if collection is not None:
                settings.prepared_shell_object = prepared_shell
                settings.prepared_static_object = prepared_static
                settings.prepared_collection = collection
                _remove_prepared(settings, restore_visibility=True)
            settings.last_prepare_status = f"Prepare failed: {exc}"
            settings.last_prepare_skipped = 0
            settings.last_seam_count = 0
            settings.last_status = settings.last_prepare_status
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            context.scene.frame_set(old_frame)


class OCS_OT_clear_prepared(Operator):
    bl_idname = "ocs.clear_prepared"
    bl_label = "Clear Prepared"
    bl_description = "Remove simulation copies created by OMP Contact Solver"

    def execute(self, context):
        if _BAKE_RUNNING:
            self.report({"ERROR"}, "A solver bake is already running")
            return {"CANCELLED"}
        removed = _remove_prepared(
            context.scene.ocs_settings,
            restore_visibility=True,
        )
        settings = context.scene.ocs_settings
        settings.last_prepare_skipped = 0
        settings.last_seam_count = 0
        settings.last_prepare_status = "Not prepared"
        settings.last_status = "Prepared copies cleared" if removed else "Nothing to clear"
        settings.last_contacts = "-"
        settings.last_residual = "-"
        settings.last_strain = "-"
        self.report({"INFO"}, settings.last_status)
        return {"FINISHED"}


class OCS_OT_clear_bake(Operator):
    bl_idname = "ocs.clear_bake"
    bl_label = "Clear Bake"
    bl_description = "Remove Shape Keys created by OMP Contact Solver"

    def execute(self, context):
        shell = context.scene.ocs_settings.prepared_shell_object
        if not _is_prepared_object(shell):
            self.report({"ERROR"}, "Run Prepare Simulation Copies first")
            return {"CANCELLED"}
        try:
            if not _clear_owned_bake(shell):
                self.report({"INFO"}, "SHELL has no solver bake")
        except RuntimeError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        context.scene.ocs_settings.last_status = "Bake cleared"
        return {"FINISHED"}


class OCS_OT_bake(Operator):
    bl_idname = "ocs.bake"
    bl_label = "Bake Simulation"
    bl_description = "Run the OpenMP solver and bake absolute Shape Keys"

    def execute(self, context):
        global _BAKE_RUNNING
        if _BAKE_RUNNING:
            self.report({"ERROR"}, "A solver bake is already running")
            return {"CANCELLED"}

        settings = context.scene.ocs_settings
        if context.mode != "OBJECT":
            self.report({"ERROR"}, "Switch Blender to Object Mode before baking")
            return {"CANCELLED"}
        try:
            shell, static = _prepared_pair(settings)
        except RuntimeError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        if settings.frame_end <= settings.frame_start:
            self.report({"ERROR"}, "End frame must be greater than start frame")
            return {"CANCELLED"}
        if shell.data.shape_keys and not _owned_bake(shell):
            self.report({"ERROR"}, "Prepared SHELL was modified; run Prepare again")
            return {"CANCELLED"}
        if abs(shell.matrix_world.determinant()) < 1.0e-12:
            self.report({"ERROR"}, "SHELL object transform is singular")
            return {"CANCELLED"}

        old_frame = context.scene.frame_current
        progress = context.window_manager
        progress.progress_begin(settings.frame_start, settings.frame_end)
        _BAKE_RUNNING = True
        bake_started = time.perf_counter()
        created_bake = False
        bake_succeeded = False

        try:
            context.scene.frame_set(settings.frame_start)
            if _owned_bake(shell):
                _clear_owned_bake(shell)

            shell_vertices, shell_triangles = _shell_mesh(shell)
            depsgraph = context.evaluated_depsgraph_get()
            (static_vertices, static_triangles, skipped_static,
             static_topology) = _filtered_static_mesh(static, depsgraph)
            seam_pairs = (
                _prepared_seam_pairs(shell) if settings.seam_enabled else []
            )
            if not shell_triangles:
                raise RuntimeError("SHELL has no triangles")
            if not static_triangles:
                raise RuntimeError("STATIC has no triangles")
            settings.last_prepare_skipped = skipped_static

            library = get_library()
            desc = library.default_desc()
            desc.gravity = Vec3(*settings.gravity)
            desc.substeps = settings.substeps
            desc.pd_iterations = settings.pd_iterations
            desc.pcg_iterations = settings.pcg_iterations
            desc.pcg_relative_tolerance = settings.pcg_tolerance
            desc.collision_iterations = settings.collision_iterations
            desc.velocity_damping = settings.velocity_damping
            desc.thread_count = settings.thread_count

            material = library.default_material()
            material.density = settings.density
            material.stretch_stiffness = settings.stretch_stiffness
            material.bend_stiffness = settings.bend_stiffness
            material.thickness = settings.thickness
            material.friction = settings.friction
            material.restitution = settings.restitution
            material.strain_limit = (
                settings.strain_limit_percent * 0.01
                if settings.strain_limit_enabled
                else 0.0
            )
            material.strain_limit_stiffness = settings.strain_limit_stiffness

            fps = context.scene.render.fps / context.scene.render.fps_base
            frame_dt = settings.time_scale / fps
            world_to_local = shell.matrix_world.inverted_safe()

            with library.create(desc) as solver:
                solver.set_static_mesh(static_vertices, static_triangles)
                solver.set_shell_mesh(shell_vertices, shell_triangles, material)
                solver.set_shell_seams(seam_pairs, settings.seam_stiffness)
                solver.build()

                basis = shell.shape_key_add(name="Basis", from_mix=False)
                basis.interpolation = "KEY_LINEAR"
                keys = shell.data.shape_keys
                keys[_BAKE_TAG] = 1
                keys["frame_start"] = settings.frame_start
                keys["frame_end"] = settings.frame_end
                created_bake = True

                final_stats = None
                for frame in range(settings.frame_start + 1, settings.frame_end + 1):
                    context.scene.frame_set(frame)
                    depsgraph = context.evaluated_depsgraph_get()
                    animated_static_vertices, topology = _evaluated_static_vertices(
                        static, depsgraph
                    )
                    if topology != static_topology:
                        raise RuntimeError(
                            "Animated STATIC topology changed; use deformation-only modifiers"
                        )
                    solver.update_static_vertices(animated_static_vertices)
                    solver.step(frame_dt)
                    shape = shell.shape_key_add(name=f"OCS_{frame:06d}", from_mix=False)
                    shape.interpolation = "KEY_LINEAR"
                    shape.data.foreach_set(
                        "co", _world_to_local_flat(world_to_local, solver.positions())
                    )
                    final_stats = solver.stats()
                    progress.progress_update(frame)

                _configure_absolute_shape_keys(
                    shell, settings.frame_start, settings.frame_end
                )

            elapsed = time.perf_counter() - bake_started
            settings.last_status = (
                f"Baked {settings.frame_end - settings.frame_start + 1} frames "
                f"with {len(seam_pairs)} seams and animated STATIC in {elapsed:.2f} s"
            )
            if final_stats is not None:
                settings.last_contacts = str(final_stats.contact_count)
                settings.last_residual = f"{final_stats.final_pcg_relative_residual:.3g}"
                settings.last_strain = (
                    f"{(final_stats.maximum_principal_stretch - 1.0) * 100.0:.2f}% "
                    f"({final_stats.strain_limit_projection_count} projections)"
                )
            context.scene.frame_set(settings.frame_start)
            bake_succeeded = True
            self.report({"INFO"}, settings.last_status)
            return {"FINISHED"}
        except Exception as exc:
            if created_bake and _owned_bake(shell):
                _clear_owned_bake(shell)
            settings.last_status = f"Bake failed: {exc}"
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            progress.progress_end()
            if not bake_succeeded:
                context.scene.frame_set(old_frame)
            _BAKE_RUNNING = False


class OCS_PT_solver(Panel):
    bl_label = "OMP Contact Solver"
    bl_idname = "OCS_PT_solver"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "OMP Cloth"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        settings = context.scene.ocs_settings

        try:
            library = get_library()
            icon = "CHECKMARK" if library.openmp_enabled else "ERROR"
            text = "DLL ready - OpenMP" if library.openmp_enabled else "DLL has no OpenMP"
        except NativeSolverError as exc:
            icon = "ERROR"
            text = str(exc)
        layout.label(text=text, icon=icon)

        objects = layout.box()
        objects.label(text="Source Objects")
        objects.prop(settings, "shell_object")
        objects.operator("ocs.set_active_shell", icon="OUTLINER_OB_MESH")
        objects.prop(settings, "static_object")
        objects.operator("ocs.set_active_static", icon="MOD_PHYSICS")
        objects.prop(settings, "static_crop_enabled")
        crop = objects.column(align=True)
        crop.enabled = settings.static_crop_enabled
        crop.prop(settings, "static_crop_min_z")
        crop.prop(settings, "static_crop_max_z")

        prepare_row = objects.row(align=True)
        prepare_row.scale_y = 1.25
        prepare_row.operator("ocs.prepare", icon="DUPLICATE")
        prepare_row.operator("ocs.clear_prepared", icon="X")

        prepared = layout.box()
        prepared.label(text="Simulation Copies")
        shell_name = (
            settings.prepared_shell_object.name
            if settings.prepared_shell_object is not None
            else "-"
        )
        static_name = (
            settings.prepared_static_object.name
            if settings.prepared_static_object is not None
            else "-"
        )
        prepared.label(text=f"SHELL: {shell_name}", icon="MESH_GRID")
        prepared.label(text=f"STATIC: {static_name}", icon="MOD_PHYSICS")
        static_modifier_count = (
            len(settings.prepared_static_object.modifiers)
            if settings.prepared_static_object is not None
            else 0
        )
        prepared.label(
            text=f"Live STATIC modifiers: {static_modifier_count}",
            icon="MODIFIER",
        )
        if settings.static_crop_enabled:
            prepared.label(
                text=f"Crop vertices: {settings.last_static_crop_vertices}",
                icon="VERTEXSEL",
            )
            prepared.label(
                text=f"Crop polygons: {settings.last_static_crop_polygons}",
                icon="FACESEL",
            )
        prepared.label(text=settings.last_prepare_status, icon="INFO")

        timing = layout.box()
        timing.label(text="Bake Range")
        row = timing.row(align=True)
        row.prop(settings, "frame_start")
        row.prop(settings, "frame_end")
        timing.prop(settings, "time_scale")

        material = layout.box()
        material.label(text="SHELL Material")
        material.prop(settings, "density")
        material.prop(settings, "stretch_stiffness")
        material.prop(settings, "bend_stiffness")
        material.prop(settings, "strain_limit_enabled")
        strain = material.column()
        strain.enabled = settings.strain_limit_enabled
        strain.prop(settings, "strain_limit_percent")
        strain.prop(settings, "strain_limit_stiffness")
        material.prop(settings, "thickness")
        material.prop(settings, "friction")
        material.prop(settings, "restitution")

        seams = layout.box()
        seams.label(text="Seam Threads")
        seams.prop(settings, "seam_enabled")
        seam_settings = seams.column()
        seam_settings.enabled = settings.seam_enabled
        seam_settings.prop(settings, "seam_search_distance")
        seam_settings.prop(settings, "seam_stiffness")
        seams.label(text=f"Detected pairs: {settings.last_seam_count}")

        solver = layout.box()
        solver.label(text="Solver")
        solver.prop(settings, "gravity")
        solver.prop(settings, "substeps")
        solver.prop(settings, "pd_iterations")
        solver.prop(settings, "pcg_iterations")
        solver.prop(settings, "pcg_tolerance")
        solver.prop(settings, "collision_iterations")
        solver.prop(settings, "velocity_damping")
        solver.prop(settings, "thread_count")

        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("ocs.bake", icon="PHYSICS")
        row.operator("ocs.clear_bake", icon="TRASH")

        status = layout.box()
        status.label(text=settings.last_status, icon="INFO")
        status.label(text=f"Last contacts: {settings.last_contacts}")
        status.label(text=f"Last PCG residual: {settings.last_residual}")
        status.label(text=f"Last max strain: {settings.last_strain}")


_CLASSES = (
    OCS_Settings,
    OCS_OT_set_active_shell,
    OCS_OT_set_active_static,
    OCS_OT_prepare,
    OCS_OT_clear_prepared,
    OCS_OT_clear_bake,
    OCS_OT_bake,
    OCS_PT_solver,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ocs_settings = PointerProperty(type=OCS_Settings)


def unregister():
    if hasattr(bpy.types.Scene, "ocs_settings"):
        del bpy.types.Scene.ocs_settings
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)

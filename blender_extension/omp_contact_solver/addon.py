"""Blender UI and bake pipeline for the OpenMP cloth DLL."""

import time

import bpy
from bpy.props import (
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Operator, Panel, PropertyGroup

from .native import NativeSolverError, Vec3, get_library


_BAKE_TAG = "omp_contact_solver_bake_version"
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


def _static_mesh(obj, depsgraph):
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    if mesh is None:
        raise RuntimeError("STATIC object could not be evaluated as a mesh")
    try:
        return _triangulated_mesh(mesh, evaluated.matrix_world)
    finally:
        evaluated.to_mesh_clear()


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
        name="SHELL",
        description="Deformable triangle mesh to simulate",
        type=bpy.types.Object,
        poll=_mesh_object,
    )
    static_object: PointerProperty(
        name="STATIC",
        description="Immutable collision mesh evaluated at the first bake frame",
        type=bpy.types.Object,
        poll=_mesh_object,
    )
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
        name="Collision Iterations", default=2, min=1, max=64
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
    thickness: FloatProperty(
        name="Thickness", default=0.01, min=0.0, max=1000.0, subtype="DISTANCE"
    )
    friction: FloatProperty(name="Friction", default=0.3, min=0.0, max=1.0)
    restitution: FloatProperty(name="Restitution", default=0.0, min=0.0, max=1.0)
    last_status: StringProperty(name="Status", default="Not baked")
    last_contacts: StringProperty(name="Contacts", default="-")
    last_residual: StringProperty(name="PCG Residual", default="-")


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
    bl_description = "Assign the active mesh as the immutable STATIC collider"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == "MESH"

    def execute(self, context):
        context.scene.ocs_settings.static_object = context.active_object
        return {"FINISHED"}


class OCS_OT_clear_bake(Operator):
    bl_idname = "ocs.clear_bake"
    bl_label = "Clear Bake"
    bl_description = "Remove Shape Keys created by OMP Contact Solver"

    def execute(self, context):
        shell = context.scene.ocs_settings.shell_object
        if shell is None:
            self.report({"ERROR"}, "Assign a SHELL mesh first")
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
        shell = settings.shell_object
        static = settings.static_object
        if context.mode != "OBJECT":
            self.report({"ERROR"}, "Switch Blender to Object Mode before baking")
            return {"CANCELLED"}
        if shell is None or static is None:
            self.report({"ERROR"}, "Assign both SHELL and STATIC mesh objects")
            return {"CANCELLED"}
        if shell == static:
            self.report({"ERROR"}, "SHELL and STATIC must be different objects")
            return {"CANCELLED"}
        if settings.frame_end <= settings.frame_start:
            self.report({"ERROR"}, "End frame must be greater than start frame")
            return {"CANCELLED"}
        if shell.data.shape_keys and not _owned_bake(shell):
            self.report({"ERROR"}, "SHELL already has Shape Keys; use an unbaked mesh copy")
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
            static_vertices, static_triangles = _static_mesh(
                static, context.evaluated_depsgraph_get()
            )
            if not shell_triangles:
                raise RuntimeError("SHELL has no triangles")
            if not static_triangles:
                raise RuntimeError("STATIC has no triangles")

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

            fps = context.scene.render.fps / context.scene.render.fps_base
            frame_dt = settings.time_scale / fps
            world_to_local = shell.matrix_world.inverted_safe()

            with library.create(desc) as solver:
                solver.set_static_mesh(static_vertices, static_triangles)
                solver.set_shell_mesh(shell_vertices, shell_triangles, material)
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
                f"in {elapsed:.2f} s"
            )
            if final_stats is not None:
                settings.last_contacts = str(final_stats.contact_count)
                settings.last_residual = f"{final_stats.final_pcg_relative_residual:.3g}"
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
        objects.label(text="Objects")
        objects.prop(settings, "shell_object")
        objects.operator("ocs.set_active_shell", icon="OUTLINER_OB_MESH")
        objects.prop(settings, "static_object")
        objects.operator("ocs.set_active_static", icon="MOD_PHYSICS")

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
        material.prop(settings, "thickness")
        material.prop(settings, "friction")
        material.prop(settings, "restitution")

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


_CLASSES = (
    OCS_Settings,
    OCS_OT_set_active_shell,
    OCS_OT_set_active_static,
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

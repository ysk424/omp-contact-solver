"""End-to-end smoke test run by Blender in background mode."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import bpy


def create_mesh_object(name, vertices, faces):
    mesh = bpy.data.meshes.new(f"{name}_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def main():
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--extension-stage", required=True)
    options = parser.parse_args(arguments)

    stage = Path(options.extension_stage).resolve()
    sys.path.insert(0, str(stage.parent))
    import omp_contact_solver
    from omp_contact_solver.native import get_library

    omp_contact_solver.register()
    try:
        assert get_library().openmp_enabled
        static = create_mesh_object(
            "OCS_Test_STATIC",
            [
                (-2, -2, 0),
                (2, -2, 0),
                (2, 2, 0),
                (-2, 2, 0),
                (10, 10, 0),
                (10.0001, 10, 0),
                (10, 10.0001, 0),
            ],
            [(0, 1, 2), (0, 2, 3), (4, 5, 6)],
        )
        shell = create_mesh_object(
            "OCS_Test_SHELL",
            [
                (-0.5, -0.5, 0.5),
                (0.5, -0.5, 0.5),
                (0.5, 0.5, 0.7),
                (-0.5, 0.5, 0.5),
            ],
            [(0, 1, 2), (0, 2, 3)],
        )
        static.location.z = 0.1
        shell.location = (0.2, -0.1, 0.3)
        shell.shape_key_add(name="User Basis")
        smooth = shell.modifiers.new(name="Evaluated Initial Shape", type="SMOOTH")
        smooth.factor = 0.5
        smooth.iterations = 1
        smooth.use_x = False
        smooth.use_y = False
        smooth.use_z = True

        depsgraph = bpy.context.evaluated_depsgraph_get()
        evaluated = shell.evaluated_get(depsgraph)
        evaluated_mesh = evaluated.to_mesh(
            preserve_all_data_layers=True,
            depsgraph=depsgraph,
        )
        try:
            expected_shell_positions = [
                tuple(evaluated.matrix_world @ vertex.co)
                for vertex in evaluated_mesh.vertices
            ]
        finally:
            evaluated.to_mesh_clear()

        settings = bpy.context.scene.ocs_settings
        settings.shell_object = shell
        settings.static_object = static
        settings.frame_start = 1
        settings.frame_end = 24
        settings.substeps = 4
        settings.thickness = 0.02

        assert bpy.ops.ocs.prepare() == {"FINISHED"}
        prepared_shell = settings.prepared_shell_object
        prepared_static = settings.prepared_static_object
        prepared_collection = settings.prepared_collection
        assert prepared_shell is not None and prepared_shell != shell
        assert prepared_static is not None and prepared_static != static
        assert prepared_collection is not None
        assert prepared_shell.name in prepared_collection.objects
        assert prepared_static.name in prepared_collection.objects
        prepared_collection_name = prepared_collection.name
        assert tuple(prepared_shell.matrix_world) == tuple(
            type(prepared_shell.matrix_world).Identity(4)
        )
        assert not prepared_shell.modifiers
        assert prepared_shell.data.shape_keys is None
        assert shell.data.shape_keys is not None
        assert shell.data.shape_keys.key_blocks[0].name == "User Basis"
        assert shell.hide_get()
        actual_shell_positions = [tuple(vertex.co) for vertex in prepared_shell.data.vertices]
        assert len(actual_shell_positions) == len(expected_shell_positions)
        for actual, expected in zip(actual_shell_positions, expected_shell_positions):
            assert max(abs(a - b) for a, b in zip(actual, expected)) < 1.0e-6
        prepared_static.data.calc_loop_triangles()
        assert len(prepared_static.data.loop_triangles) == 2
        assert settings.last_prepare_skipped == 1

        assert bpy.ops.ocs.bake() == {"FINISHED"}
        keys = prepared_shell.data.shape_keys
        assert keys is not None
        assert keys.get("omp_contact_solver_bake_version") == 1
        assert len(keys.key_blocks) == 24
        assert keys.use_relative is False
        final_height = min(
            (prepared_shell.matrix_world @ point.co).z
            for point in keys.key_blocks[-1].data
        )
        assert 0.119 <= final_height <= 0.2, final_height
        assert settings.last_status.startswith("Baked 24 frames")

        assert bpy.ops.ocs.clear_bake() == {"FINISHED"}
        assert prepared_shell.data.shape_keys is None
        assert shell.data.shape_keys is not None
        assert shell.data.shape_keys.key_blocks[0].name == "User Basis"

        assert bpy.ops.ocs.clear_prepared() == {"FINISHED"}
        assert settings.prepared_shell_object is None
        assert settings.prepared_static_object is None
        assert bpy.data.collections.get(prepared_collection_name) is None
        assert not shell.hide_get()
        print(
            "Blender Extension preparation smoke test passed: "
            f"OpenMP={get_library().openmp_enabled}, skipped_static=1, "
            f"final_min_z={final_height:.6f}"
        )
    finally:
        omp_contact_solver.unregister()


if __name__ == "__main__":
    main()

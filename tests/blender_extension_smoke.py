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
            [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)],
            [(0, 1, 2), (0, 2, 3)],
        )
        shell = create_mesh_object(
            "OCS_Test_SHELL",
            [(-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)],
            [(0, 1, 2), (0, 2, 3)],
        )
        static.location.z = 0.1
        shell.location = (0.2, -0.1, 0.3)
        settings = bpy.context.scene.ocs_settings
        settings.shell_object = shell
        settings.static_object = static
        settings.frame_start = 1
        settings.frame_end = 24
        settings.substeps = 4
        settings.thickness = 0.02

        assert bpy.ops.ocs.bake() == {"FINISHED"}
        keys = shell.data.shape_keys
        assert keys is not None
        assert keys.get("omp_contact_solver_bake_version") == 1
        assert len(keys.key_blocks) == 24
        assert keys.use_relative is False
        final_height = min(
            (shell.matrix_world @ point.co).z for point in keys.key_blocks[-1].data
        )
        assert 0.119 <= final_height <= 0.2, final_height
        assert settings.last_status.startswith("Baked 24 frames")

        assert bpy.ops.ocs.clear_bake() == {"FINISHED"}
        assert shell.data.shape_keys is None

        shell.shape_key_add(name="User Basis")
        try:
            result = bpy.ops.ocs.bake()
        except RuntimeError as exc:
            assert "already has Shape Keys" in str(exc)
        else:
            assert result == {"CANCELLED"}
        assert shell.data.shape_keys is not None
        assert shell.data.shape_keys.key_blocks[0].name == "User Basis"
        shell.shape_key_clear()
        print(
            "Blender Extension smoke test passed: "
            f"OpenMP={get_library().openmp_enabled}, final_min_z={final_height:.6f}"
        )
    finally:
        omp_contact_solver.unregister()


if __name__ == "__main__":
    main()

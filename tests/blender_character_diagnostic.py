"""Run the Extension on a real character without saving the opened blend."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    return ordered[min(int((len(ordered) - 1) * fraction), len(ordered) - 1)]


def edge_metrics(shell):
    rest = [tuple(vertex.co) for vertex in shell.data.vertices]
    final = [tuple(point.co) for point in shell.data.shape_keys.key_blocks[-1].data]
    ratios = []
    for edge in shell.data.edges:
        a, b = edge.vertices
        rest_length = math.dist(rest[a], rest[b])
        if rest_length > 1.0e-12:
            ratios.append(math.dist(final[a], final[b]) / rest_length)

    usage = {}
    shell.data.calc_loop_triangles()
    for triangle in shell.data.loop_triangles:
        a, b, c = triangle.vertices
        for edge in ((a, b), (b, c), (c, a)):
            key = tuple(sorted(edge))
            usage[key] = usage.get(key, 0) + 1
    z_values = [point[2] for point in rest]
    upper_cut = min(z_values) + 0.72 * (max(z_values) - min(z_values))
    upper_boundary = [
        edge
        for edge, count in usage.items()
        if count == 1 and max(rest[edge[0]][2], rest[edge[1]][2]) >= upper_cut
    ]
    rest_boundary = sum(math.dist(rest[a], rest[b]) for a, b in upper_boundary)
    final_boundary = sum(math.dist(final[a], final[b]) for a, b in upper_boundary)
    return {
        "edge_q95": percentile(ratios, 0.95),
        "edge_q99": percentile(ratios, 0.99),
        "edge_max": max(ratios, default=1.0),
        "upper_boundary_length_ratio": (
            final_boundary / rest_boundary if rest_boundary > 0.0 else 1.0
        ),
    }


def main():
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--extension-stage", required=True)
    parser.add_argument("--blend", required=True)
    parser.add_argument("--shell", required=True)
    parser.add_argument("--static", required=True)
    parser.add_argument("--frame-end", type=int, default=30)
    parser.add_argument("--crop-min", type=float, default=0.40)
    parser.add_argument("--crop-max", type=float, default=1.45)
    parser.add_argument("--thickness", type=float, default=0.01)
    parser.add_argument("--substeps", type=int, default=10)
    parser.add_argument("--pd-iterations", type=int, default=8)
    parser.add_argument("--pcg-iterations", type=int, default=64)
    parser.add_argument("--collision-safety-passes", type=int, default=0)
    parser.add_argument("--strain-stiffness", type=float, default=100000.0)
    options = parser.parse_args(arguments)

    bpy.ops.wm.open_mainfile(filepath=str(Path(options.blend).resolve()))
    stage = Path(options.extension_stage).resolve()
    sys.path.insert(0, str(stage.parent))
    import omp_contact_solver

    omp_contact_solver.register()
    try:
        scene = bpy.context.scene
        settings = scene.ocs_settings
        settings.shell_object = bpy.data.objects[options.shell]
        settings.static_object = bpy.data.objects[options.static]
        settings.frame_start = 1
        settings.frame_end = options.frame_end
        settings.static_crop_enabled = True
        settings.static_crop_min_z = options.crop_min
        settings.static_crop_max_z = options.crop_max
        settings.strain_limit_enabled = True
        settings.strain_limit_percent = 5.0
        settings.strain_limit_stiffness = options.strain_stiffness
        settings.thickness = options.thickness
        settings.substeps = options.substeps
        settings.pd_iterations = options.pd_iterations
        settings.pcg_iterations = options.pcg_iterations
        settings.collision_iterations = options.collision_safety_passes
        if bpy.ops.ocs.prepare() != {"FINISHED"}:
            raise RuntimeError(settings.last_prepare_status)
        prepared_static = settings.prepared_static_object
        depsgraph = bpy.context.evaluated_depsgraph_get()
        evaluated = prepared_static.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh(depsgraph=depsgraph)
        try:
            static_counts = {
                "vertices": len(mesh.vertices),
                "triangles": len(mesh.loop_triangles),
            }
        finally:
            evaluated.to_mesh_clear()
        if bpy.ops.ocs.bake() != {"FINISHED"}:
            raise RuntimeError(settings.last_status)
        result = {
            "crop_vertices": settings.last_static_crop_vertices,
            "crop_polygons": settings.last_static_crop_polygons,
            "static": static_counts,
            "native_max_strain": settings.last_strain,
            "contacts": settings.last_contacts,
            "pcg_residual": settings.last_residual,
            **edge_metrics(settings.prepared_shell_object),
        }
        print("OCS_CHARACTER_DIAGNOSTIC=" + json.dumps(result, sort_keys=True))
    finally:
        omp_contact_solver.unregister()


if __name__ == "__main__":
    main()

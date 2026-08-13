#include "omp_contact_solver.h"

#include <cstdio>
#include <vector>

int main() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.thread_count = 0; // all OpenMP threads available to this process
    OcsSolver *solver = ocsCreate(&desc);
    if (!solver) {
        std::fprintf(stderr, "create: %s\n", ocsGetLastError(nullptr));
        return 1;
    }

    /* A floor plus a low pyramid. The final y range demonstrates deformation;
       this program does not render anything. */
    const OcsVec3 static_vertices[] = {
        {-2, 0, -2}, {2, 0, -2}, {2, 0, 2}, {-2, 0, 2},
        {-0.35f, 0, -0.35f}, {0.35f, 0, -0.35f},
        {0.35f, 0, 0.35f}, {-0.35f, 0, 0.35f}, {0, 0.55f, 0}};
    const OcsTriangle static_triangles[] = {
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 8}, {5, 6, 8}, {6, 7, 8}, {7, 4, 8}};
    if (ocsSetStaticMesh(solver, static_vertices, 9, static_triangles, 6) != OCS_OK) {
        std::fprintf(stderr, "STATIC: %s\n", ocsGetLastError(solver));
        ocsDestroy(solver);
        return 1;
    }

    constexpr uint32_t side = 24;
    std::vector<OcsVec3> vertices;
    std::vector<OcsTriangle> triangles;
    vertices.reserve(side * side);
    for (uint32_t z = 0; z < side; ++z) {
        for (uint32_t x = 0; x < side; ++x) {
            vertices.push_back({-0.75f + 1.5f * x / (side - 1),
                                1.2f,
                                -0.75f + 1.5f * z / (side - 1)});
        }
    }
    for (uint32_t z = 0; z + 1 < side; ++z) {
        for (uint32_t x = 0; x + 1 < side; ++x) {
            const uint32_t a = z * side + x;
            const uint32_t b = a + 1;
            const uint32_t c = a + side;
            const uint32_t d = c + 1;
            triangles.push_back({a, c, b});
            triangles.push_back({b, c, d});
        }
    }
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    material.thickness = 0.015f;
    if (ocsSetShellMesh(solver, vertices.data(), static_cast<uint32_t>(vertices.size()),
                        triangles.data(), static_cast<uint32_t>(triangles.size()),
                        &material) != OCS_OK || ocsBuild(solver) != OCS_OK) {
        std::fprintf(stderr, "SHELL/build: %s\n", ocsGetLastError(solver));
        ocsDestroy(solver);
        return 1;
    }

    for (int frame = 0; frame < 120; ++frame) {
        if (ocsStep(solver, 1.0f / 60.0f) != OCS_OK) {
            std::fprintf(stderr, "step: %s\n", ocsGetLastError(solver));
            ocsDestroy(solver);
            return 1;
        }
    }
    ocsCopyShellPositions(solver, vertices.data(), static_cast<uint32_t>(vertices.size()));
    float min_y = vertices.front().y;
    float max_y = vertices.front().y;
    for (const OcsVec3 p : vertices) {
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }
    OcsStepStats stats{};
    stats.struct_size = sizeof(stats);
    ocsGetLastStepStats(solver, &stats);
    std::printf("vertices=%u y=[%.5f, %.5f] contacts=%llu pcg_iterations=%llu\n",
                ocsGetShellVertexCount(solver), min_y, max_y,
                static_cast<unsigned long long>(stats.contact_count),
                static_cast<unsigned long long>(stats.pcg_iterations));
    ocsDestroy(solver);
    return 0;
}

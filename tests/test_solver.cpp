#include "omp_contact_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

void require(bool condition, const char *message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

void require_ok(OcsResult result, OcsSolver *solver, const char *operation) {
    if (result != OCS_OK) {
        std::fprintf(stderr, "FAIL: %s: %s\n", operation, ocsGetLastError(solver));
        std::exit(1);
    }
}

float distance(OcsVec3 a, OcsVec3 b) {
    const float x = a.x - b.x;
    const float y = a.y - b.y;
    const float z = a.z - b.z;
    return std::sqrt(x * x + y * y + z * z);
}

void test_free_fall() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.substeps = 2;
    desc.pd_iterations = 4;
    desc.thread_count = 2;
    OcsSolver *solver = ocsCreate(&desc);
    require(solver != nullptr, "create free-fall solver");

    const OcsVec3 vertices[] = {
        {-0.5f, 1.0f, 0.0f}, {0.5f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.8f}};
    const OcsTriangle triangles[] = {{0, 1, 2}};
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    require_ok(ocsSetShellMesh(solver, vertices, 3, triangles, 1, &material),
               solver, "set free-fall SHELL");
    require_ok(ocsBuild(solver), solver, "build free-fall solver");
    require_ok(ocsStep(solver, 1.0f / 60.0f), solver, "free-fall step");

    OcsVec3 result[3];
    require_ok(ocsCopyShellPositions(solver, result, 3), solver,
               "copy free-fall positions");
    require(result[0].y < vertices[0].y, "gravity must move SHELL downward");
    require(std::abs(distance(result[0], result[1]) - 1.0f) < 2.0e-3f,
            "stretch constraint must preserve a rest edge");
    ocsDestroy(solver);
}

void test_static_floor_contact() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.substeps = 4;
    desc.pd_iterations = 6;
    desc.pcg_iterations = 60;
    desc.thread_count = 4;
    OcsSolver *solver = ocsCreate(&desc);
    require(solver != nullptr, "create contact solver");

    const OcsVec3 floor_vertices[] = {
        {-3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, -3.0f},
        {3.0f, 0.0f, 3.0f}, {-3.0f, 0.0f, 3.0f}};
    const OcsTriangle floor_triangles[] = {{0, 2, 1}, {0, 3, 2}};
    require_ok(ocsSetStaticMesh(solver, floor_vertices, 4,
                                floor_triangles, 2),
               solver, "set STATIC floor");

    const OcsVec3 shell_vertices[] = {
        {-0.5f, 0.7f, -0.5f}, {0.5f, 0.7f, -0.5f},
        {0.5f, 0.7f, 0.5f}, {-0.5f, 0.7f, 0.5f}};
    const OcsTriangle shell_triangles[] = {{0, 1, 2}, {0, 2, 3}};
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    material.thickness = 0.02f;
    material.friction = 0.6f;
    require_ok(ocsSetShellMesh(solver, shell_vertices, 4,
                               shell_triangles, 2, &material),
               solver, "set contact SHELL");
    require_ok(ocsBuild(solver), solver, "build contact solver");

    for (int frame = 0; frame < 150; ++frame) {
        require_ok(ocsStep(solver, 1.0f / 60.0f), solver, "contact step");
    }
    OcsVec3 result[4];
    OcsVec3 velocity[4];
    require_ok(ocsCopyShellPositions(solver, result, 4), solver,
               "copy contact positions");
    require_ok(ocsCopyShellVelocities(solver, velocity, 4), solver,
               "copy contact velocities");
    float minimum_y = result[0].y;
    float maximum_speed = 0.0f;
    for (int i = 0; i < 4; ++i) {
        minimum_y = std::min(minimum_y, result[i].y);
        maximum_speed = std::max(maximum_speed,
            std::sqrt(velocity[i].x * velocity[i].x +
                      velocity[i].y * velocity[i].y +
                      velocity[i].z * velocity[i].z));
    }
    require(minimum_y >= material.thickness * 0.95f,
            "SHELL vertices must remain above STATIC by thickness");
    require(maximum_speed < 0.25f, "resting SHELL should not gain energy");

    OcsStepStats stats{};
    stats.struct_size = sizeof(stats);
    require_ok(ocsGetLastStepStats(solver, &stats), solver, "get step stats");
    require(stats.contact_count > 0, "resting frame must report contacts");
    ocsDestroy(solver);
}

void test_seam_thread() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.substeps = 4;
    desc.pd_iterations = 10;
    desc.pcg_iterations = 200;

    const OcsVec3 floor_vertices[] = {
        {-3.0f, 0.0f, -3.0f}, {3.0f, 0.0f, -3.0f},
        {3.0f, 0.0f, 3.0f}, {-3.0f, 0.0f, 3.0f}};
    const OcsTriangle floor_triangles[] = {{0, 2, 1}, {0, 3, 2}};

    const OcsVec3 shell_vertices[] = {
        {-0.2f, 0.02f, -0.2f}, {0.2f, 0.02f, -0.2f},
        {0.0f, 0.02f, 0.2f},
        {-0.2f, 1.0f, -0.2f}, {0.2f, 1.0f, -0.2f},
        {0.0f, 1.0f, 0.2f}};
    const OcsTriangle shell_triangles[] = {{0, 1, 2}, {3, 4, 5}};
    const float rest_length = distance(shell_vertices[0], shell_vertices[3]);

    auto seam_error = [&](float stiffness) {
        OcsSolver *solver = ocsCreate(&desc);
        require(solver != nullptr, "create seam solver");
        require_ok(ocsSetStaticMesh(solver, floor_vertices, 4,
                                    floor_triangles, 2),
                   solver, "set seam STATIC floor");
        OcsShellMaterial material;
        ocsDefaultShellMaterial(&material);
        material.thickness = 0.02f;
        require_ok(ocsSetShellMesh(solver, shell_vertices, 6,
                                   shell_triangles, 2, &material),
                   solver, "set disconnected seam SHELL");
        const OcsSeam seams[] = {{0, 3, stiffness}};
        require_ok(ocsSetShellSeams(solver, seams, 1), solver,
                   "set finite seam");
        require_ok(ocsBuild(solver), solver, "build seam solver");
        for (int frame = 0; frame < 10; ++frame) {
            require_ok(ocsStep(solver, 1.0f / 60.0f), solver, "seam step");
        }
        OcsVec3 result[6];
        require_ok(ocsCopyShellPositions(solver, result, 6), solver,
                   "copy seam positions");
        const float error =
            std::abs(distance(result[0], result[3]) - rest_length);
        ocsDestroy(solver);
        return error;
    };

    const float soft_error = seam_error(100.0f);
    const float strong_error = seam_error(100000.0f);
    require(strong_error < soft_error * 0.25f,
            "higher finite seam stiffness must reduce seam strain");
    require(strong_error < rest_length * 0.05f,
            "strong finite seam must keep strain below five percent");
}

void test_triangle_strain_limit() {
    const OcsVec3 wall_vertices[] = {
        {0.0f, -2.0f, -2.0f}, {0.0f, 2.0f, -2.0f},
        {0.0f, 2.0f, 2.0f}, {0.0f, -2.0f, 2.0f}};
    const OcsTriangle wall_triangles[] = {{0, 1, 2}, {0, 2, 3}};
    const OcsVec3 shell_vertices[] = {
        {-0.001f, -0.5f, 0.0f}, {0.5f, -0.5f, 0.0f},
        {0.5f, 0.5f, 0.0f}};
    const OcsTriangle shell_triangle[] = {{0, 1, 2}};

    auto run = [&](float limit, float stiffness, uint64_t *projections) {
        OcsSolverDesc desc;
        ocsDefaultSolverDesc(&desc);
        desc.gravity = {0.0f, 0.0f, 0.0f};
        desc.substeps = 4;
        desc.pd_iterations = 8;
        desc.pcg_iterations = 300;
        desc.thread_count = 2;
        OcsSolver *solver = ocsCreate(&desc);
        require(solver != nullptr, "create strain-limit solver");
        require_ok(ocsSetStaticMesh(solver, wall_vertices, 4,
                                    wall_triangles, 2),
                   solver, "set strain-limit wall");
        OcsShellMaterial material;
        ocsDefaultShellMaterial(&material);
        material.stretch_stiffness = 1.0f;
        material.bend_stiffness = 0.0f;
        material.thickness = 0.2f;
        material.strain_limit = limit;
        material.strain_limit_stiffness = stiffness;
        require_ok(ocsSetShellMesh(solver, shell_vertices, 3,
                                   shell_triangle, 1, &material),
                   solver, "set strain-limit SHELL");
        require_ok(ocsBuild(solver), solver, "build strain-limit solver");
        OcsStepStats stats{};
        for (int frame = 0; frame < 10; ++frame) {
            require_ok(ocsStep(solver, 1.0f / 24.0f), solver,
                       "strain-limit step");
            stats.struct_size = sizeof(stats);
            require_ok(ocsGetLastStepStats(solver, &stats), solver,
                       "strain-limit stats");
            *projections += stats.strain_limit_projection_count;
        }
        const float maximum = stats.maximum_principal_stretch;
        ocsDestroy(solver);
        return maximum;
    };

    uint64_t disabled_projections = 0u;
    uint64_t enabled_projections = 0u;
    const float unlimited = run(0.0f, 0.0f, &disabled_projections);
    const float limited = run(0.05f, 10000.0f, &enabled_projections);
    require(unlimited > 2.0f,
            "stress scene must visibly stretch without the limiter");
    require(limited <= 1.06f,
            "five-percent triangle strain limit must bound principal stretch");
    require(limited < unlimited * 0.5f,
            "triangle strain limit must materially reduce stretch");
    require(disabled_projections == 0u && enabled_projections > 0u,
            "strain-limit statistics must report active projections");
}

void test_invalid_mesh() {
    OcsSolver *solver = ocsCreate(nullptr);
    require(solver != nullptr, "create invalid-mesh solver");
    const OcsVec3 vertices[] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    const OcsTriangle triangle[] = {{0, 1, 2}};
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    require_ok(ocsSetShellMesh(solver, vertices, 3, triangle, 1, &material),
               solver, "accept arrays before validation");
    require(ocsBuild(solver) == OCS_ERROR_INVALID_MESH,
            "degenerate SHELL must fail at build");
    require(ocsGetLastError(solver)[0] != '\0', "mesh failure must have an error message");
    ocsDestroy(solver);
}

void test_openmp_sized_mesh() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.substeps = 1;
    desc.pd_iterations = 2;
    desc.pcg_iterations = 20;
    desc.thread_count = 4;
    OcsSolver *solver = ocsCreate(&desc);
    require(solver != nullptr, "create OpenMP-sized solver");

    constexpr uint32_t side = 66; // 4,356 vertices: exceeds parallel threshold.
    std::vector<OcsVec3> vertices;
    std::vector<OcsTriangle> triangles;
    vertices.reserve(side * side);
    for (uint32_t z = 0; z < side; ++z) {
        for (uint32_t x = 0; x < side; ++x) {
            vertices.push_back({0.02f * x, 2.0f, 0.02f * z});
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
    require_ok(ocsSetShellMesh(solver, vertices.data(),
                               static_cast<uint32_t>(vertices.size()),
                               triangles.data(),
                               static_cast<uint32_t>(triangles.size()),
                               &material),
               solver, "set OpenMP-sized SHELL");
    require_ok(ocsBuild(solver), solver, "build OpenMP-sized solver");
    require_ok(ocsStep(solver, 1.0f / 60.0f), solver, "OpenMP-sized step");
    require_ok(ocsCopyShellPositions(solver, vertices.data(),
                                     static_cast<uint32_t>(vertices.size())),
               solver, "copy OpenMP-sized positions");
    require(vertices.front().y < 2.0f &&
            std::abs(vertices.front().y - vertices.back().y) < 1.0e-4f,
            "parallel free fall must remain finite and uniform");
    ocsDestroy(solver);
}

void test_swept_floor_contact() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.substeps = 1;
    desc.pd_iterations = 3;
    desc.gravity = {0.0f, -30.0f, 0.0f};
    OcsSolver *solver = ocsCreate(&desc);
    require(solver != nullptr, "create swept-contact solver");

    const OcsVec3 floor_vertices[] = {
        {-5, 0, -5}, {5, 0, -5}, {5, 0, 5}, {-5, 0, 5}};
    const OcsTriangle floor_triangles[] = {{0, 2, 1}, {0, 3, 2}};
    require_ok(ocsSetStaticMesh(solver, floor_vertices, 4, floor_triangles, 2),
               solver, "set swept STATIC");

    const OcsVec3 shell_vertices[] = {
        {-0.2f, 1.0f, -0.2f}, {0.2f, 1.0f, -0.2f}, {0.0f, 1.0f, 0.2f}};
    const OcsTriangle shell_triangle[] = {{0, 1, 2}};
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    material.thickness = 0.025f;
    require_ok(ocsSetShellMesh(solver, shell_vertices, 3, shell_triangle, 1,
                               &material),
               solver, "set swept SHELL");
    require_ok(ocsBuild(solver), solver, "build swept-contact solver");
    /* The unconstrained predictor ends far below y=0. A discrete endpoint-only
       query would miss the plane; the swept vertex-triangle query must catch it. */
    require_ok(ocsStep(solver, 0.5f), solver, "swept-contact step");
    OcsVec3 result[3];
    require_ok(ocsCopyShellPositions(solver, result, 3), solver,
               "copy swept-contact positions");
    for (OcsVec3 p : result) {
        require(p.y >= material.thickness * 0.95f,
                "swept contact must prevent high-speed vertex tunnelling");
    }
    ocsDestroy(solver);
}

void test_animated_static_refit() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.gravity = {0.0f, 0.0f, 0.0f};
    desc.substeps = 4;
    desc.pd_iterations = 4;
    OcsSolver *solver = ocsCreate(&desc);
    require(solver != nullptr, "create animated-STATIC solver");

    const OcsVec3 floor_start[] = {
        {-2.0f, -0.02f, -2.0f}, {2.0f, -0.02f, -2.0f},
        {2.0f, -0.02f, 2.0f}, {-2.0f, -0.02f, 2.0f}};
    const OcsVec3 floor_end[] = {
        {-2.0f, 0.0f, -2.0f}, {2.0f, 0.0f, -2.0f},
        {2.0f, 0.0f, 2.0f}, {-2.0f, 0.0f, 2.0f}};
    const OcsTriangle floor_triangles[] = {{0, 2, 1}, {0, 3, 2}};
    require_ok(ocsSetStaticMesh(solver, floor_start, 4, floor_triangles, 2),
               solver, "set animated STATIC start");

    const OcsVec3 shell_vertices[] = {
        {-0.2f, 0.0f, -0.2f}, {0.2f, 0.0f, -0.2f},
        {0.0f, 0.0f, 0.2f}};
    const OcsTriangle shell_triangle[] = {{0, 1, 2}};
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    material.thickness = 0.02f;
    require_ok(ocsSetShellMesh(solver, shell_vertices, 3, shell_triangle, 1,
                               &material),
               solver, "set animated-STATIC SHELL");
    require_ok(ocsBuild(solver), solver, "build animated-STATIC solver");
    require_ok(ocsUpdateStaticVertices(solver, floor_end, 4), solver,
               "queue animated STATIC vertices");
    require_ok(ocsStep(solver, 1.0f / 24.0f), solver,
               "step animated STATIC");

    OcsVec3 result[3];
    require_ok(ocsCopyShellPositions(solver, result, 3), solver,
               "copy animated-STATIC positions");
    for (OcsVec3 point : result) {
        require(point.y >= material.thickness * 0.95f,
                "rising STATIC must carry SHELL to its final surface");
    }

    OcsVec3 temporarily_degenerate[] = {
        floor_end[0], floor_end[1], floor_end[0], floor_end[3]};
    require_ok(ocsUpdateStaticVertices(solver, temporarily_degenerate, 4), solver,
               "queue temporarily degenerate STATIC");
    require_ok(ocsStep(solver, 1.0f / 24.0f), solver,
               "ignore temporarily degenerate STATIC triangles");
    require_ok(ocsUpdateStaticVertices(solver, floor_end, 4), solver,
               "reactivate animated STATIC triangles");
    require_ok(ocsStep(solver, 1.0f / 24.0f), solver,
               "step reactivated STATIC triangles");
    ocsDestroy(solver);
}

} // namespace

int main() {
    require(ocsGetAbiVersion() == OCS_ABI_VERSION, "ABI version");
    require(ocsIsOpenMpEnabled() == 1, "library must be compiled with OpenMP");
    test_free_fall();
    test_static_floor_contact();
    test_seam_thread();
    test_triangle_strain_limit();
    test_invalid_mesh();
    test_openmp_sized_mesh();
    test_swept_floor_contact();
    test_animated_static_refit();
    std::puts("All omp-contact-solver tests passed.");
    return 0;
}

#include "solver.hpp"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstring>
#include <exception>
#include <new>

namespace {

thread_local std::string g_last_error;

void set_global_error(const char *message) {
    g_last_error = message ? message : "unknown error";
}

OcsResult fail(OcsSolver *solver, OcsResult result, const char *message) {
    if (solver) {
        solver->impl.set_error(message ? message : "unknown error");
    }
    set_global_error(message);
    return result;
}

bool valid_solver_desc(const OcsSolverDesc &d) {
    return d.struct_size == sizeof(OcsSolverDesc) && d.substeps > 0 &&
           d.pd_iterations > 0 && d.pcg_iterations > 0 &&
           std::isfinite(d.gravity.x) && std::isfinite(d.gravity.y) &&
           std::isfinite(d.gravity.z) &&
           std::isfinite(d.pcg_relative_tolerance) &&
           d.pcg_relative_tolerance > 0.0f &&
           d.collision_iterations > 0 && std::isfinite(d.velocity_damping) &&
           d.velocity_damping >= 0.0f && d.thread_count <= INT_MAX;
}

bool valid_material(const OcsShellMaterial &m) {
    return m.struct_size == sizeof(OcsShellMaterial) &&
           std::isfinite(m.density) && std::isfinite(m.stretch_stiffness) &&
           std::isfinite(m.bend_stiffness) && std::isfinite(m.thickness) &&
           std::isfinite(m.friction) && std::isfinite(m.restitution) &&
           m.density > 0.0f &&
           m.stretch_stiffness > 0.0f && m.bend_stiffness >= 0.0f &&
           m.thickness >= 0.0f && m.friction >= 0.0f && m.friction <= 1.0f &&
           m.restitution >= 0.0f && m.restitution <= 1.0f;
}

} // namespace

extern "C" {

uint32_t ocsGetAbiVersion(void) { return OCS_ABI_VERSION; }

int32_t ocsIsOpenMpEnabled(void) {
#ifdef _OPENMP
    return 1;
#else
    return 0;
#endif
}

void ocsDefaultSolverDesc(OcsSolverDesc *desc) {
    if (!desc) return;
    std::memset(desc, 0, sizeof(*desc));
    desc->struct_size = sizeof(*desc);
    desc->gravity = {0.0f, -9.81f, 0.0f};
    desc->substeps = 4;
    desc->pd_iterations = 8;
    desc->pcg_iterations = 80;
    desc->pcg_relative_tolerance = 1.0e-5f;
    desc->collision_iterations = 2;
    desc->velocity_damping = 0.05f;
    desc->thread_count = 0;
}

void ocsDefaultShellMaterial(OcsShellMaterial *material) {
    if (!material) return;
    std::memset(material, 0, sizeof(*material));
    material->struct_size = sizeof(*material);
    material->density = 1.0f;
    material->stretch_stiffness = 1000.0f;
    material->bend_stiffness = 1.0f;
    material->thickness = 0.01f;
    material->friction = 0.35f;
    material->restitution = 0.0f;
}

OcsSolver *ocsCreate(const OcsSolverDesc *desc) {
    OcsSolverDesc value{};
    if (desc) {
        value = *desc;
    } else {
        ocsDefaultSolverDesc(&value);
    }
    if (!valid_solver_desc(value)) {
        set_global_error("invalid OcsSolverDesc");
        return nullptr;
    }
    try {
        OcsSolver *solver = new OcsSolver(value);
        g_last_error.clear();
        return solver;
    } catch (const std::bad_alloc &) {
        set_global_error("out of memory while creating solver");
    } catch (const std::exception &e) {
        set_global_error(e.what());
    } catch (...) {
        set_global_error("unknown exception while creating solver");
    }
    return nullptr;
}

void ocsDestroy(OcsSolver *solver) { delete solver; }

OcsResult ocsSetStaticMesh(OcsSolver *solver, const OcsVec3 *vertices,
                           uint32_t vertex_count,
                           const OcsTriangle *triangles,
                           uint32_t triangle_count) {
    if (!solver) return fail(nullptr, OCS_ERROR_INVALID_ARGUMENT, "solver is null");
    try {
        return solver->impl.set_static_mesh(vertices, vertex_count, triangles,
                                            triangle_count)
                   ? OCS_OK
                   : OCS_ERROR_INVALID_MESH;
    } catch (const std::bad_alloc &) {
        return fail(solver, OCS_ERROR_OUT_OF_MEMORY, "out of memory while setting STATIC mesh");
    } catch (const std::exception &e) {
        return fail(solver, OCS_ERROR_INTERNAL, e.what());
    } catch (...) {
        return fail(solver, OCS_ERROR_INTERNAL, "unknown error while setting STATIC mesh");
    }
}

OcsResult ocsSetShellMesh(OcsSolver *solver, const OcsVec3 *vertices,
                          uint32_t vertex_count,
                          const OcsTriangle *triangles,
                          uint32_t triangle_count,
                          const OcsShellMaterial *material) {
    if (!solver || !material) {
        return fail(solver, OCS_ERROR_INVALID_ARGUMENT, "solver or material is null");
    }
    if (!valid_material(*material)) {
        return fail(solver, OCS_ERROR_INVALID_ARGUMENT, "invalid OcsShellMaterial");
    }
    try {
        return solver->impl.set_shell_mesh(vertices, vertex_count, triangles,
                                           triangle_count, *material)
                   ? OCS_OK
                   : OCS_ERROR_INVALID_MESH;
    } catch (const std::bad_alloc &) {
        return fail(solver, OCS_ERROR_OUT_OF_MEMORY, "out of memory while setting SHELL mesh");
    } catch (const std::exception &e) {
        return fail(solver, OCS_ERROR_INTERNAL, e.what());
    } catch (...) {
        return fail(solver, OCS_ERROR_INTERNAL, "unknown error while setting SHELL mesh");
    }
}

OcsResult ocsSetShellSeams(OcsSolver *solver, const OcsSeam *seams,
                           uint32_t seam_count) {
    if (!solver) {
        return fail(nullptr, OCS_ERROR_INVALID_ARGUMENT, "solver is null");
    }
    try {
        return solver->impl.set_shell_seams(seams, seam_count)
                   ? OCS_OK
                   : OCS_ERROR_INVALID_MESH;
    } catch (const std::bad_alloc &) {
        return fail(solver, OCS_ERROR_OUT_OF_MEMORY,
                    "out of memory while setting SHELL seams");
    } catch (const std::exception &e) {
        return fail(solver, OCS_ERROR_INTERNAL, e.what());
    } catch (...) {
        return fail(solver, OCS_ERROR_INTERNAL,
                    "unknown error while setting SHELL seams");
    }
}

OcsResult ocsBuild(OcsSolver *solver) {
    if (!solver) return fail(nullptr, OCS_ERROR_INVALID_ARGUMENT, "solver is null");
    try {
        return solver->impl.build() ? OCS_OK : OCS_ERROR_INVALID_MESH;
    } catch (const std::bad_alloc &) {
        return fail(solver, OCS_ERROR_OUT_OF_MEMORY, "out of memory while building solver");
    } catch (const std::exception &e) {
        return fail(solver, OCS_ERROR_INTERNAL, e.what());
    } catch (...) {
        return fail(solver, OCS_ERROR_INTERNAL, "unknown error while building solver");
    }
}

OcsResult ocsStep(OcsSolver *solver, float frame_dt) {
    if (!solver || !(frame_dt > 0.0f)) {
        return fail(solver, OCS_ERROR_INVALID_ARGUMENT, "solver is null or frame_dt is not positive");
    }
    if (!solver->impl.built()) {
        return fail(solver, OCS_ERROR_INVALID_STATE, "ocsBuild must succeed before ocsStep");
    }
    try {
        return solver->impl.step(frame_dt) ? OCS_OK : OCS_ERROR_NUMERICAL_FAILURE;
    } catch (const std::bad_alloc &) {
        return fail(solver, OCS_ERROR_OUT_OF_MEMORY, "out of memory during step");
    } catch (const std::exception &e) {
        return fail(solver, OCS_ERROR_INTERNAL, e.what());
    } catch (...) {
        return fail(solver, OCS_ERROR_INTERNAL, "unknown error during step");
    }
}

uint32_t ocsGetShellVertexCount(const OcsSolver *solver) {
    return solver ? solver->impl.vertex_count() : 0u;
}

OcsResult ocsCopyShellPositions(const OcsSolver *solver, OcsVec3 *positions,
                                uint32_t capacity) {
    if (!solver || !positions || capacity < solver->impl.vertex_count()) {
        return fail(const_cast<OcsSolver *>(solver), OCS_ERROR_INVALID_ARGUMENT,
                    "position output buffer is null or too small");
    }
    const auto &src = solver->impl.positions();
    for (size_t i = 0; i < src.size(); ++i) {
        positions[i] = {src[i].x, src[i].y, src[i].z};
    }
    return OCS_OK;
}

OcsResult ocsCopyShellVelocities(const OcsSolver *solver, OcsVec3 *velocities,
                                 uint32_t capacity) {
    if (!solver || !velocities || capacity < solver->impl.vertex_count()) {
        return fail(const_cast<OcsSolver *>(solver), OCS_ERROR_INVALID_ARGUMENT,
                    "velocity output buffer is null or too small");
    }
    const auto &src = solver->impl.velocities();
    for (size_t i = 0; i < src.size(); ++i) {
        velocities[i] = {src[i].x, src[i].y, src[i].z};
    }
    return OCS_OK;
}

OcsResult ocsGetLastStepStats(const OcsSolver *solver, OcsStepStats *stats) {
    if (!solver || !stats || stats->struct_size != sizeof(OcsStepStats)) {
        return fail(const_cast<OcsSolver *>(solver), OCS_ERROR_INVALID_ARGUMENT,
                    "invalid OcsStepStats output");
    }
    const ocs::StepStats &src = solver->impl.stats();
    stats->substeps = src.substeps;
    stats->pd_iterations = src.pd_iterations;
    stats->pcg_iterations = src.pcg_iterations;
    stats->contact_count = src.contacts;
    stats->final_pcg_relative_residual = src.residual;
    return OCS_OK;
}

const char *ocsGetLastError(const OcsSolver *solver) {
    return solver ? solver->impl.error().c_str() : g_last_error.c_str();
}

} // extern "C"

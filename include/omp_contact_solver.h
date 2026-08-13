#ifndef OMP_CONTACT_SOLVER_H
#define OMP_CONTACT_SOLVER_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(OCS_BUILD_DLL)
#    define OCS_API __declspec(dllexport)
#  else
#    define OCS_API __declspec(dllimport)
#  endif
#else
#  define OCS_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define OCS_ABI_VERSION 2u

typedef struct OcsSolver OcsSolver;

typedef struct OcsVec3 {
    float x;
    float y;
    float z;
} OcsVec3;

typedef struct OcsTriangle {
    uint32_t i0;
    uint32_t i1;
    uint32_t i2;
} OcsTriangle;

/* A non-stretching thread between two SHELL vertices. Rest length is captured
 * from the SHELL positions during ocsBuild(). */
typedef struct OcsSeam {
    uint32_t i0;
    uint32_t i1;
    float stiffness;
} OcsSeam;

typedef enum OcsResult {
    OCS_OK = 0,
    OCS_ERROR_INVALID_ARGUMENT = 1,
    OCS_ERROR_INVALID_STATE = 2,
    OCS_ERROR_INVALID_MESH = 3,
    OCS_ERROR_OUT_OF_MEMORY = 4,
    OCS_ERROR_NUMERICAL_FAILURE = 5,
    OCS_ERROR_INTERNAL = 6
} OcsResult;

/* Solver-wide settings. Set struct_size with ocsDefaultSolverDesc(). */
typedef struct OcsSolverDesc {
    uint32_t struct_size;
    OcsVec3 gravity;
    uint32_t substeps;
    uint32_t pd_iterations;
    uint32_t pcg_iterations;
    float pcg_relative_tolerance;
    uint32_t collision_iterations;
    float velocity_damping;
    uint32_t thread_count; /* 0 selects omp_get_max_threads(). */
} OcsSolverDesc;

/* One material is applied to the complete SHELL mesh. */
typedef struct OcsShellMaterial {
    uint32_t struct_size;
    float density;
    float stretch_stiffness;
    float bend_stiffness;
    float thickness;
    float friction;
    float restitution;
} OcsShellMaterial;

typedef struct OcsStepStats {
    uint32_t struct_size;
    uint32_t substeps;
    uint32_t pd_iterations;
    uint64_t pcg_iterations;
    uint64_t contact_count;
    float final_pcg_relative_residual;
} OcsStepStats;

OCS_API uint32_t ocsGetAbiVersion(void);
OCS_API int32_t ocsIsOpenMpEnabled(void);
OCS_API void ocsDefaultSolverDesc(OcsSolverDesc *desc);
OCS_API void ocsDefaultShellMaterial(OcsShellMaterial *material);

OCS_API OcsSolver *ocsCreate(const OcsSolverDesc *desc);
OCS_API void ocsDestroy(OcsSolver *solver);

/* STATIC is immutable after ocsBuild(). Passing zero triangles clears it. */
OCS_API OcsResult ocsSetStaticMesh(OcsSolver *solver,
                                   const OcsVec3 *vertices,
                                   uint32_t vertex_count,
                                   const OcsTriangle *triangles,
                                   uint32_t triangle_count);

/* Exactly one simulated SHELL mesh is supported. PIN constraints are absent. */
OCS_API OcsResult ocsSetShellMesh(OcsSolver *solver,
                                  const OcsVec3 *vertices,
                                  uint32_t vertex_count,
                                  const OcsTriangle *triangles,
                                  uint32_t triangle_count,
                                  const OcsShellMaterial *material);

/* Optional seam-thread constraints. Call after ocsSetShellMesh() and before
 * ocsBuild(). Passing zero seams clears them. */
OCS_API OcsResult ocsSetShellSeams(OcsSolver *solver,
                                   const OcsSeam *seams,
                                   uint32_t seam_count);

/* Builds SHELL constraints and the immutable STATIC triangle BVH. */
OCS_API OcsResult ocsBuild(OcsSolver *solver);

/* Advances by frame_dt seconds. The configured substeps are internal. */
OCS_API OcsResult ocsStep(OcsSolver *solver, float frame_dt);

OCS_API uint32_t ocsGetShellVertexCount(const OcsSolver *solver);
OCS_API OcsResult ocsCopyShellPositions(const OcsSolver *solver,
                                        OcsVec3 *positions,
                                        uint32_t capacity);
OCS_API OcsResult ocsCopyShellVelocities(const OcsSolver *solver,
                                         OcsVec3 *velocities,
                                         uint32_t capacity);
OCS_API OcsResult ocsGetLastStepStats(const OcsSolver *solver,
                                      OcsStepStats *stats);

/* The returned pointer remains valid until the next call on this solver. */
OCS_API const char *ocsGetLastError(const OcsSolver *solver);

#ifdef __cplusplus
}
#endif

#endif

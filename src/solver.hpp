#ifndef OCS_SOLVER_HPP
#define OCS_SOLVER_HPP

#include "omp_contact_solver.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ocs {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 a, float s);
Vec3 operator*(float s, Vec3 a);
Vec3 operator/(Vec3 a, float s);
Vec3 &operator+=(Vec3 &a, Vec3 b);
Vec3 &operator-=(Vec3 &a, Vec3 b);
Vec3 &operator*=(Vec3 &a, float s);
float dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);
float length_squared(Vec3 a);
float length(Vec3 a);

struct Aabb {
    Vec3 lo;
    Vec3 hi;
};

struct StaticTriangle {
    uint32_t i[3]{};
    Vec3 normal;
    Vec3 centroid;
    Aabb bounds;
};

struct BvhNode {
    Aabb bounds;
    uint32_t first = 0;
    uint32_t count = 0;
    uint32_t left = UINT32_MAX;
    uint32_t right = UINT32_MAX;
};

class StaticBvh {
public:
    bool build(const std::vector<Vec3> &vertices,
               const std::vector<OcsTriangle> &triangles,
               std::string &error);
    bool empty() const { return triangles_.empty(); }

    struct ClosestHit {
        bool hit = false;
        float distance_squared = 0.0f;
        Vec3 point;
        Vec3 normal;
    };

    struct SegmentHit {
        bool hit = false;
        float time = 1.0f;
        Vec3 point;
        Vec3 normal;
    };

    ClosestHit closest_within(Vec3 point, float radius) const;
    SegmentHit first_segment_hit(Vec3 from, Vec3 to, float padding) const;

private:
    uint32_t build_node(uint32_t first, uint32_t count);

    std::vector<Vec3> vertices_;
    std::vector<StaticTriangle> triangles_;
    std::vector<uint32_t> order_;
    std::vector<BvhNode> nodes_;
};

struct Constraint {
    uint32_t a = 0;
    uint32_t b = 0;
    float rest_length = 0.0f;
    float weight = 0.0f;
};

struct Incidence {
    uint32_t constraint = 0;
    uint32_t other = 0;
    float sign = 1.0f;
};

struct StepStats {
    uint32_t substeps = 0;
    uint32_t pd_iterations = 0;
    uint64_t pcg_iterations = 0;
    uint64_t contacts = 0;
    float residual = 0.0f;
};

class Solver {
public:
    explicit Solver(const OcsSolverDesc &desc);

    bool set_static_mesh(const OcsVec3 *vertices, uint32_t vertex_count,
                         const OcsTriangle *triangles, uint32_t triangle_count);
    bool set_shell_mesh(const OcsVec3 *vertices, uint32_t vertex_count,
                        const OcsTriangle *triangles, uint32_t triangle_count,
                        const OcsShellMaterial &material);
    bool set_shell_seams(const OcsSeam *seams, uint32_t seam_count);
    bool build();
    bool step(float frame_dt);

    uint32_t vertex_count() const { return static_cast<uint32_t>(positions_.size()); }
    const std::vector<Vec3> &positions() const { return positions_; }
    const std::vector<Vec3> &velocities() const { return velocities_; }
    const std::string &error() const { return error_; }
    const StepStats &stats() const { return stats_; }
    bool built() const { return built_; }
    void set_error(std::string message) { error_ = std::move(message); }

private:
    bool build_shell_constraints();
    bool solve_pcg(const std::vector<Vec3> &rhs, float inv_h2,
                   std::vector<Vec3> &x);
    void apply_system(const std::vector<Vec3> &x, float inv_h2,
                      std::vector<Vec3> &out) const;
    double parallel_dot(const std::vector<Vec3> &a,
                        const std::vector<Vec3> &b) const;
    uint64_t resolve_collisions(const std::vector<Vec3> &from,
                                std::vector<Vec3> &positions,
                                std::vector<Vec3> &contact_normals,
                                std::vector<uint8_t> &contacted) const;
    void project_seams(std::vector<Vec3> &positions) const;
    bool finite_state() const;

    OcsSolverDesc desc_{};
    OcsShellMaterial material_{};
    int threads_ = 1;
    bool shell_set_ = false;
    bool built_ = false;

    std::vector<Vec3> static_vertices_;
    std::vector<OcsTriangle> static_triangles_;
    std::vector<OcsTriangle> shell_triangles_;
    std::vector<OcsSeam> shell_seams_;
    std::vector<Vec3> rest_positions_;
    std::vector<Vec3> positions_;
    std::vector<Vec3> velocities_;
    std::vector<float> masses_;
    std::vector<Constraint> constraints_;
    std::vector<Constraint> seam_constraints_;
    std::vector<uint32_t> incidence_offsets_;
    std::vector<Incidence> incidence_;
    StaticBvh static_bvh_;

    std::vector<Vec3> projection_;
    std::vector<Vec3> rhs_;
    std::vector<Vec3> pcg_r_;
    std::vector<Vec3> pcg_z_;
    std::vector<Vec3> pcg_p_;
    std::vector<Vec3> pcg_ap_;
    std::vector<float> pcg_diag_;
    std::vector<Vec3> substep_start_;
    std::vector<Vec3> predicted_;
    std::vector<Vec3> iterate_;
    std::vector<Vec3> contact_normals_;
    std::vector<uint8_t> contacted_;

    StepStats stats_{};
    std::string error_;
};

} // namespace ocs

struct OcsSolver {
    explicit OcsSolver(const OcsSolverDesc &desc) : impl(desc) {}
    ocs::Solver impl;
};

#endif

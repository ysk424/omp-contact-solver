#include "solver.hpp"

#include <omp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace ocs {

namespace {

constexpr float kEpsilon = 1.0e-7f;
constexpr uint32_t kLeafTriangles = 8;
/* Below this size, OpenMP launch and reduction overhead costs more than the
 * arithmetic. Larger production meshes still take every parallel path. */
constexpr int64_t kParallelThreshold = 4096;

Vec3 from_public(OcsVec3 v) { return {v.x, v.y, v.z}; }

bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Vec3 min_vec(Vec3 a, Vec3 b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

Vec3 max_vec(Vec3 a, Vec3 b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

Aabb empty_aabb() {
    const float inf = std::numeric_limits<float>::infinity();
    return {{inf, inf, inf}, {-inf, -inf, -inf}};
}

void grow(Aabb &a, Vec3 p) {
    a.lo = min_vec(a.lo, p);
    a.hi = max_vec(a.hi, p);
}

void grow(Aabb &a, const Aabb &b) {
    a.lo = min_vec(a.lo, b.lo);
    a.hi = max_vec(a.hi, b.hi);
}

Aabb expanded(Aabb a, float amount) {
    const Vec3 e{amount, amount, amount};
    a.lo -= e;
    a.hi += e;
    return a;
}

float aabb_distance_squared(const Aabb &a, Vec3 p) {
    const float dx = std::max(std::max(a.lo.x - p.x, 0.0f), p.x - a.hi.x);
    const float dy = std::max(std::max(a.lo.y - p.y, 0.0f), p.y - a.hi.y);
    const float dz = std::max(std::max(a.lo.z - p.z, 0.0f), p.z - a.hi.z);
    return dx * dx + dy * dy + dz * dz;
}

bool segment_aabb(Vec3 p0, Vec3 p1, const Aabb &box, float max_time) {
    const Vec3 d = p1 - p0;
    float tmin = 0.0f;
    float tmax = max_time;
    const float origin[3] = {p0.x, p0.y, p0.z};
    const float delta[3] = {d.x, d.y, d.z};
    const float lo[3] = {box.lo.x, box.lo.y, box.lo.z};
    const float hi[3] = {box.hi.x, box.hi.y, box.hi.z};
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(delta[axis]) < kEpsilon) {
            if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) return false;
            continue;
        }
        const float inv = 1.0f / delta[axis];
        float a = (lo[axis] - origin[axis]) * inv;
        float b = (hi[axis] - origin[axis]) * inv;
        if (a > b) std::swap(a, b);
        tmin = std::max(tmin, a);
        tmax = std::min(tmax, b);
        if (tmin > tmax) return false;
    }
    return true;
}

Vec3 closest_point_triangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;
    const float d1 = dot(ab, ap);
    const float d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp);
    const float d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp);
    const float d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const Vec3 bc = c - b;
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + bc * w;
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

bool segment_triangle(Vec3 p0, Vec3 p1, Vec3 a, Vec3 b, Vec3 c,
                      float &time) {
    const Vec3 d = p1 - p0;
    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 p = cross(d, e2);
    const float determinant = dot(e1, p);
    if (std::abs(determinant) < kEpsilon) return false;
    const float inv = 1.0f / determinant;
    const Vec3 t = p0 - a;
    const float u = dot(t, p) * inv;
    if (u < -kEpsilon || u > 1.0f + kEpsilon) return false;
    const Vec3 q = cross(t, e1);
    const float v = dot(d, q) * inv;
    if (v < -kEpsilon || u + v > 1.0f + kEpsilon) return false;
    const float hit_time = dot(e2, q) * inv;
    if (hit_time < kEpsilon || hit_time > time) return false;
    time = hit_time;
    return true;
}

uint64_t edge_key(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(a) << 32u) | static_cast<uint64_t>(b);
}

float component(Vec3 v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

} // namespace

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
Vec3 operator*(float s, Vec3 a) { return a * s; }
Vec3 operator/(Vec3 a, float s) { return {a.x / s, a.y / s, a.z / s}; }
Vec3 &operator+=(Vec3 &a, Vec3 b) { a = a + b; return a; }
Vec3 &operator-=(Vec3 &a, Vec3 b) { a = a - b; return a; }
Vec3 &operator*=(Vec3 &a, float s) { a = a * s; return a; }
float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
float length_squared(Vec3 a) { return dot(a, a); }
float length(Vec3 a) { return std::sqrt(length_squared(a)); }

bool StaticBvh::build(const std::vector<Vec3> &vertices,
                      const std::vector<OcsTriangle> &triangles,
                      std::string &error) {
    vertices_ = vertices;
    triangles_.clear();
    order_.clear();
    nodes_.clear();
    if (triangles.empty()) return true;
    triangles_.reserve(triangles.size());
    for (size_t ti = 0; ti < triangles.size(); ++ti) {
        const OcsTriangle t = triangles[ti];
        if (t.i0 >= vertices.size() || t.i1 >= vertices.size() ||
            t.i2 >= vertices.size() || t.i0 == t.i1 || t.i1 == t.i2 ||
            t.i2 == t.i0) {
            error = "STATIC contains an invalid triangle index";
            return false;
        }
        const Vec3 a = vertices[t.i0];
        const Vec3 b = vertices[t.i1];
        const Vec3 c = vertices[t.i2];
        const Vec3 n_raw = cross(b - a, c - a);
        const float n_len = length(n_raw);
        if (!(n_len > kEpsilon)) {
            error = "STATIC contains a degenerate triangle";
            return false;
        }
        StaticTriangle out;
        out.i[0] = t.i0;
        out.i[1] = t.i1;
        out.i[2] = t.i2;
        out.normal = n_raw / n_len;
        out.centroid = (a + b + c) / 3.0f;
        out.bounds = empty_aabb();
        grow(out.bounds, a);
        grow(out.bounds, b);
        grow(out.bounds, c);
        triangles_.push_back(out);
    }
    order_.resize(triangles_.size());
    std::iota(order_.begin(), order_.end(), 0u);
    nodes_.reserve(triangles_.size() * 2u);
    build_node(0u, static_cast<uint32_t>(order_.size()));
    return true;
}

uint32_t StaticBvh::build_node(uint32_t first, uint32_t count) {
    const uint32_t node_index = static_cast<uint32_t>(nodes_.size());
    nodes_.push_back({});
    Aabb bounds = empty_aabb();
    Aabb centroids = empty_aabb();
    for (uint32_t i = first; i < first + count; ++i) {
        const StaticTriangle &t = triangles_[order_[i]];
        grow(bounds, t.bounds);
        grow(centroids, t.centroid);
    }
    nodes_[node_index].bounds = bounds;
    nodes_[node_index].first = first;
    nodes_[node_index].count = count;
    if (count <= kLeafTriangles) return node_index;

    const Vec3 extent = centroids.hi - centroids.lo;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (component(extent, 2) > component(extent, axis)) axis = 2;
    const uint32_t middle = first + count / 2u;
    std::nth_element(order_.begin() + first, order_.begin() + middle,
                     order_.begin() + first + count,
                     [&](uint32_t lhs, uint32_t rhs) {
                         return component(triangles_[lhs].centroid, axis) <
                                component(triangles_[rhs].centroid, axis);
                     });
    const uint32_t left = build_node(first, middle - first);
    const uint32_t right = build_node(middle, first + count - middle);
    nodes_[node_index].left = left;
    nodes_[node_index].right = right;
    nodes_[node_index].count = 0;
    return node_index;
}

StaticBvh::ClosestHit StaticBvh::closest_within(Vec3 point, float radius) const {
    ClosestHit result;
    result.distance_squared = radius * radius;
    if (nodes_.empty()) return result;
    std::array<uint32_t, 128> stack{};
    size_t stack_size = 0;
    stack[stack_size++] = 0u;
    while (stack_size != 0u) {
        const uint32_t node_index = stack[--stack_size];
        const BvhNode &node = nodes_[node_index];
        if (aabb_distance_squared(node.bounds, point) > result.distance_squared) continue;
        if (node.count != 0u) {
            for (uint32_t i = node.first; i < node.first + node.count; ++i) {
                const StaticTriangle &t = triangles_[order_[i]];
                const Vec3 q = closest_point_triangle(
                    point, vertices_[t.i[0]], vertices_[t.i[1]], vertices_[t.i[2]]);
                const float d2 = length_squared(point - q);
                if (d2 <= result.distance_squared) {
                    result.hit = true;
                    result.distance_squared = d2;
                    result.point = q;
                    result.normal = t.normal;
                }
            }
        } else {
            stack[stack_size++] = node.left;
            stack[stack_size++] = node.right;
        }
    }
    return result;
}

StaticBvh::SegmentHit StaticBvh::first_segment_hit(Vec3 from, Vec3 to,
                                                    float padding) const {
    SegmentHit result;
    if (nodes_.empty() || length_squared(to - from) <= kEpsilon * kEpsilon) return result;
    std::array<uint32_t, 128> stack{};
    size_t stack_size = 0;
    stack[stack_size++] = 0u;
    while (stack_size != 0u) {
        const uint32_t node_index = stack[--stack_size];
        const BvhNode &node = nodes_[node_index];
        if (!segment_aabb(from, to, expanded(node.bounds, padding), result.time)) continue;
        if (node.count != 0u) {
            for (uint32_t i = node.first; i < node.first + node.count; ++i) {
                const StaticTriangle &t = triangles_[order_[i]];
                float hit_time = result.time;
                if (segment_triangle(from, to, vertices_[t.i[0]], vertices_[t.i[1]],
                                     vertices_[t.i[2]], hit_time)) {
                    result.hit = true;
                    result.time = hit_time;
                    result.point = from + (to - from) * hit_time;
                    result.normal = t.normal;
                }
            }
        } else {
            stack[stack_size++] = node.left;
            stack[stack_size++] = node.right;
        }
    }
    return result;
}

Solver::Solver(const OcsSolverDesc &desc) : desc_(desc) {
    threads_ = desc.thread_count == 0u ? omp_get_max_threads()
                                       : static_cast<int>(desc.thread_count);
    threads_ = std::max(1, threads_);
}

bool Solver::set_static_mesh(const OcsVec3 *vertices, uint32_t vertex_count,
                             const OcsTriangle *triangles,
                             uint32_t triangle_count) {
    error_.clear();
    if (built_) {
        error_ = "meshes cannot be changed after ocsBuild";
        return false;
    }
    if (triangle_count == 0u) {
        static_vertices_.clear();
        static_triangles_.clear();
        return true;
    }
    if (!vertices || !triangles || vertex_count == 0u) {
        error_ = "STATIC arrays are null or empty";
        return false;
    }
    static_vertices_.resize(vertex_count);
    for (uint32_t i = 0; i < vertex_count; ++i) {
        static_vertices_[i] = from_public(vertices[i]);
        if (!finite(static_vertices_[i])) {
            error_ = "STATIC contains a non-finite vertex";
            return false;
        }
    }
    static_triangles_.assign(triangles, triangles + triangle_count);
    return true;
}

bool Solver::set_shell_mesh(const OcsVec3 *vertices, uint32_t vertex_count,
                            const OcsTriangle *triangles,
                            uint32_t triangle_count,
                            const OcsShellMaterial &material) {
    error_.clear();
    if (built_) {
        error_ = "meshes cannot be changed after ocsBuild";
        return false;
    }
    if (!vertices || !triangles || vertex_count < 3u || triangle_count == 0u) {
        error_ = "SHELL arrays are null or empty";
        return false;
    }
    rest_positions_.resize(vertex_count);
    for (uint32_t i = 0; i < vertex_count; ++i) {
        rest_positions_[i] = from_public(vertices[i]);
        if (!finite(rest_positions_[i])) {
            error_ = "SHELL contains a non-finite vertex";
            return false;
        }
    }
    positions_ = rest_positions_;
    velocities_.assign(vertex_count, {});
    shell_triangles_.assign(triangles, triangles + triangle_count);
    material_ = material;
    shell_set_ = true;
    return true;
}

bool Solver::build() {
    error_.clear();
    if (built_) {
        error_ = "ocsBuild may only be called once";
        return false;
    }
    if (!shell_set_) {
        error_ = "one SHELL mesh is required";
        return false;
    }
    if (!static_bvh_.build(static_vertices_, static_triangles_, error_)) return false;
    if (!build_shell_constraints()) return false;
    const size_t n = positions_.size();
    projection_.resize(constraints_.size());
    rhs_.resize(n);
    pcg_r_.resize(n);
    pcg_z_.resize(n);
    pcg_p_.resize(n);
    pcg_ap_.resize(n);
    pcg_diag_.resize(n);
    substep_start_.resize(n);
    predicted_.resize(n);
    iterate_.resize(n);
    contact_normals_.resize(n);
    contacted_.resize(n);
    built_ = true;
    return true;
}

bool Solver::build_shell_constraints() {
    struct EdgeRecord {
        uint32_t a = 0;
        uint32_t b = 0;
        uint32_t opposite[2]{};
        uint32_t count = 0;
    };
    std::unordered_map<uint64_t, EdgeRecord> edges;
    edges.reserve(shell_triangles_.size() * 3u);
    masses_.assign(positions_.size(), 0.0f);

    auto add_edge = [&](uint32_t a, uint32_t b, uint32_t opposite) -> bool {
        const uint64_t key = edge_key(a, b);
        auto it = edges.find(key);
        if (it == edges.end()) {
            EdgeRecord rec;
            rec.a = std::min(a, b);
            rec.b = std::max(a, b);
            rec.opposite[0] = opposite;
            rec.count = 1u;
            edges.emplace(key, rec);
            return true;
        }
        if (it->second.count >= 2u) {
            error_ = "SHELL has a non-manifold edge shared by more than two triangles";
            return false;
        }
        it->second.opposite[it->second.count++] = opposite;
        return true;
    };

    for (const OcsTriangle &t : shell_triangles_) {
        if (t.i0 >= positions_.size() || t.i1 >= positions_.size() ||
            t.i2 >= positions_.size() || t.i0 == t.i1 || t.i1 == t.i2 ||
            t.i2 == t.i0) {
            error_ = "SHELL contains an invalid triangle index";
            return false;
        }
        const Vec3 a = rest_positions_[t.i0];
        const Vec3 b = rest_positions_[t.i1];
        const Vec3 c = rest_positions_[t.i2];
        const float twice_area = length(cross(b - a, c - a));
        if (!(twice_area > kEpsilon)) {
            error_ = "SHELL contains a degenerate triangle";
            return false;
        }
        const float lump = material_.density * (0.5f * twice_area) / 3.0f;
        masses_[t.i0] += lump;
        masses_[t.i1] += lump;
        masses_[t.i2] += lump;
        if (!add_edge(t.i0, t.i1, t.i2) || !add_edge(t.i1, t.i2, t.i0) ||
            !add_edge(t.i2, t.i0, t.i1)) return false;
    }
    for (float mass : masses_) {
        if (!(mass > 0.0f) || !std::isfinite(mass)) {
            error_ = "every SHELL vertex must belong to a non-degenerate triangle";
            return false;
        }
    }

    constraints_.clear();
    constraints_.reserve(edges.size() * 2u);
    for (const auto &entry : edges) {
        const EdgeRecord &e = entry.second;
        const float rest = length(rest_positions_[e.a] - rest_positions_[e.b]);
        if (!(rest > kEpsilon)) {
            error_ = "SHELL contains a zero-length edge";
            return false;
        }
        constraints_.push_back({e.a, e.b, rest, material_.stretch_stiffness});
        if (e.count == 2u && material_.bend_stiffness > 0.0f &&
            e.opposite[0] != e.opposite[1]) {
            const uint32_t oa = e.opposite[0];
            const uint32_t ob = e.opposite[1];
            const float bend_rest = length(rest_positions_[oa] - rest_positions_[ob]);
            if (bend_rest > kEpsilon) {
                constraints_.push_back({oa, ob, bend_rest, material_.bend_stiffness});
            }
        }
    }

    incidence_offsets_.assign(positions_.size() + 1u, 0u);
    for (const Constraint &c : constraints_) {
        ++incidence_offsets_[c.a + 1u];
        ++incidence_offsets_[c.b + 1u];
    }
    for (size_t i = 1; i < incidence_offsets_.size(); ++i) {
        incidence_offsets_[i] += incidence_offsets_[i - 1u];
    }
    incidence_.resize(constraints_.size() * 2u);
    std::vector<uint32_t> cursor = incidence_offsets_;
    for (uint32_t ci = 0; ci < constraints_.size(); ++ci) {
        const Constraint &c = constraints_[ci];
        incidence_[cursor[c.a]++] = {ci, c.b, 1.0f};
        incidence_[cursor[c.b]++] = {ci, c.a, -1.0f};
    }
    return true;
}

void Solver::apply_system(const std::vector<Vec3> &x, float inv_h2,
                          std::vector<Vec3> &out) const {
    const int64_t n = static_cast<int64_t>(x.size());
#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
    for (int64_t vi = 0; vi < n; ++vi) {
        Vec3 value = x[vi] * (masses_[vi] * inv_h2);
        for (uint32_t k = incidence_offsets_[vi];
             k < incidence_offsets_[vi + 1u]; ++k) {
            const Incidence &inc = incidence_[k];
            const Constraint &c = constraints_[inc.constraint];
            value += (x[vi] - x[inc.other]) * c.weight;
        }
        out[vi] = value;
    }
}

double Solver::parallel_dot(const std::vector<Vec3> &a,
                            const std::vector<Vec3> &b) const {
    const int64_t n = static_cast<int64_t>(a.size());
    double sum = 0.0;
#pragma omp parallel for schedule(static) reduction(+ : sum) num_threads(threads_) if(n > kParallelThreshold)
    for (int64_t i = 0; i < n; ++i) {
        sum += static_cast<double>(a[i].x) * b[i].x +
               static_cast<double>(a[i].y) * b[i].y +
               static_cast<double>(a[i].z) * b[i].z;
    }
    return sum;
}

bool Solver::solve_pcg(const std::vector<Vec3> &rhs, float inv_h2,
                       std::vector<Vec3> &x) {
    const int64_t n = static_cast<int64_t>(x.size());
    apply_system(x, inv_h2, pcg_ap_);
#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
    for (int64_t i = 0; i < n; ++i) {
        float diagonal = masses_[i] * inv_h2;
        for (uint32_t k = incidence_offsets_[i];
             k < incidence_offsets_[i + 1u]; ++k) {
            const Incidence &inc = incidence_[k];
            diagonal += constraints_[inc.constraint].weight;
        }
        pcg_diag_[i] = diagonal;
        pcg_r_[i] = rhs[i] - pcg_ap_[i];
        pcg_z_[i] = pcg_r_[i] / diagonal;
        pcg_p_[i] = pcg_z_[i];
    }
    const double rhs_norm = std::sqrt(std::max(parallel_dot(rhs, rhs), 1.0e-30));
    double rz = parallel_dot(pcg_r_, pcg_z_);
    double relative = std::sqrt(std::max(parallel_dot(pcg_r_, pcg_r_), 0.0)) /
                      rhs_norm;
    if (!std::isfinite(relative) || !std::isfinite(rz)) {
        error_ = "PCG received a non-finite residual";
        return false;
    }
    if (relative <= desc_.pcg_relative_tolerance) {
        stats_.residual = static_cast<float>(relative);
        return true;
    }

    for (uint32_t iteration = 0; iteration < desc_.pcg_iterations; ++iteration) {
        apply_system(pcg_p_, inv_h2, pcg_ap_);
        const double denominator = parallel_dot(pcg_p_, pcg_ap_);
        if (!(denominator > 1.0e-30) || !std::isfinite(denominator)) {
            error_ = "PCG system is not positive definite";
            return false;
        }
        const float alpha = static_cast<float>(rz / denominator);
#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
        for (int64_t i = 0; i < n; ++i) {
            x[i] += pcg_p_[i] * alpha;
            pcg_r_[i] -= pcg_ap_[i] * alpha;
        }
        ++stats_.pcg_iterations;
        relative = std::sqrt(std::max(parallel_dot(pcg_r_, pcg_r_), 0.0)) /
                   rhs_norm;
        if (!std::isfinite(relative)) {
            error_ = "PCG produced a non-finite residual";
            return false;
        }
        if (relative <= desc_.pcg_relative_tolerance) break;

#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
        for (int64_t i = 0; i < n; ++i) pcg_z_[i] = pcg_r_[i] / pcg_diag_[i];
        const double rz_next = parallel_dot(pcg_r_, pcg_z_);
        if (!std::isfinite(rz_next)) {
            error_ = "PCG preconditioner produced a non-finite value";
            return false;
        }
        const float beta = static_cast<float>(rz_next / rz);
#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
        for (int64_t i = 0; i < n; ++i) {
            pcg_p_[i] = pcg_z_[i] + pcg_p_[i] * beta;
        }
        rz = rz_next;
    }
    stats_.residual = static_cast<float>(relative);
    return true;
}

uint64_t Solver::resolve_collisions(const std::vector<Vec3> &from,
                                    std::vector<Vec3> &positions,
                                    std::vector<Vec3> &contact_normals,
                                    std::vector<uint8_t> &contacted) const {
    if (static_bvh_.empty()) return 0u;
    const int64_t n = static_cast<int64_t>(positions.size());
    const float thickness = std::max(material_.thickness, 1.0e-6f);
    const float skin = std::max(1.0e-5f, thickness * 1.0e-3f);
    uint64_t contacts = 0u;
#pragma omp parallel for schedule(static) reduction(+ : contacts) num_threads(threads_) if(n > kParallelThreshold)
    for (int64_t vi = 0; vi < n; ++vi) {
        Vec3 p = positions[vi];
        Vec3 normal{};
        bool has_contact = false;

        const StaticBvh::SegmentHit crossing =
            static_bvh_.first_segment_hit(from[vi], p, thickness);
        if (crossing.hit) {
            float side = dot(from[vi] - crossing.point, crossing.normal);
            if (std::abs(side) < kEpsilon) {
                side = -dot(p - from[vi], crossing.normal);
            }
            normal = side >= 0.0f ? crossing.normal : crossing.normal * -1.0f;
            p = crossing.point + normal * (thickness + skin);
            has_contact = true;
        }

        const StaticBvh::ClosestHit nearest =
            static_bvh_.closest_within(p, thickness + skin);
        if (nearest.hit && nearest.distance_squared < thickness * thickness) {
            const Vec3 offset = p - nearest.point;
            const float distance = length(offset);
            Vec3 push_normal;
            if (distance > kEpsilon) {
                push_normal = offset / distance;
            } else {
                float side = dot(from[vi] - nearest.point, nearest.normal);
                push_normal = side >= 0.0f ? nearest.normal : nearest.normal * -1.0f;
            }
            p += push_normal * (thickness + skin - distance);
            normal = push_normal;
            has_contact = true;
        }

        positions[vi] = p;
        if (has_contact) {
            contact_normals[vi] = normal;
            contacted[vi] = 1u;
            contacts += 1u;
        }
    }
    return contacts;
}

bool Solver::step(float frame_dt) {
    error_.clear();
    stats_ = {};
    stats_.substeps = desc_.substeps;
    stats_.pd_iterations = desc_.substeps * desc_.pd_iterations;
    const float h = frame_dt / static_cast<float>(desc_.substeps);
    if (!(h > 0.0f) || !std::isfinite(h)) {
        error_ = "invalid substep duration";
        return false;
    }
    const float inv_h2 = 1.0f / (h * h);
    const float damping = std::exp(-desc_.velocity_damping * h);
    const int64_t n = static_cast<int64_t>(positions_.size());
    const int64_t nc = static_cast<int64_t>(constraints_.size());
    for (uint32_t substep = 0; substep < desc_.substeps; ++substep) {
        substep_start_ = positions_;
        std::fill(contact_normals_.begin(), contact_normals_.end(), Vec3{});
        std::fill(contacted_.begin(), contacted_.end(), 0u);
#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
        for (int64_t i = 0; i < n; ++i) {
            velocities_[i] *= damping;
            predicted_[i] = positions_[i] + velocities_[i] * h +
                            from_public(desc_.gravity) * (h * h);
            iterate_[i] = predicted_[i];
        }

        for (uint32_t pd = 0; pd < desc_.pd_iterations; ++pd) {
#pragma omp parallel for schedule(static) num_threads(threads_) if(nc > kParallelThreshold)
            for (int64_t ci = 0; ci < nc; ++ci) {
                const Constraint &c = constraints_[ci];
                const Vec3 delta = iterate_[c.a] - iterate_[c.b];
                const float len = length(delta);
                projection_[ci] = len > kEpsilon
                                      ? delta * (c.weight * c.rest_length / len)
                                      : (rest_positions_[c.a] - rest_positions_[c.b]) *
                                            (c.weight * c.rest_length /
                                             std::max(length(rest_positions_[c.a] -
                                                             rest_positions_[c.b]),
                                                      kEpsilon));
            }
#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
            for (int64_t vi = 0; vi < n; ++vi) {
                Vec3 value = predicted_[vi] * (masses_[vi] * inv_h2);
                for (uint32_t k = incidence_offsets_[vi];
                     k < incidence_offsets_[vi + 1u]; ++k) {
                    const Incidence &inc = incidence_[k];
                    value += projection_[inc.constraint] * inc.sign;
                }
                rhs_[vi] = value;
            }
            if (!solve_pcg(rhs_, inv_h2, iterate_)) return false;
            for (uint32_t collision_pass = 0;
                 collision_pass < desc_.collision_iterations; ++collision_pass) {
                stats_.contacts +=
                    resolve_collisions(substep_start_, iterate_, contact_normals_, contacted_);
            }
        }

#pragma omp parallel for schedule(static) num_threads(threads_) if(n > kParallelThreshold)
        for (int64_t i = 0; i < n; ++i) {
            Vec3 velocity = (iterate_[i] - substep_start_[i]) / h;
            if (contacted_[i]) {
                const Vec3 normal = contact_normals_[i];
                const float vn = dot(velocity, normal);
                if (vn < 0.0f) {
                    velocity -= normal * ((1.0f + material_.restitution) * vn);
                }
                const float normal_speed = dot(velocity, normal);
                const Vec3 tangent = velocity - normal * normal_speed;
                velocity -= tangent * material_.friction;
            }
            positions_[i] = iterate_[i];
            velocities_[i] = velocity;
        }
    }
    if (!finite_state()) {
        error_ = "solver produced a non-finite SHELL state";
        return false;
    }
    return true;
}

bool Solver::finite_state() const {
    for (size_t i = 0; i < positions_.size(); ++i) {
        if (!finite(positions_[i]) || !finite(velocities_[i])) return false;
    }
    return true;
}

} // namespace ocs

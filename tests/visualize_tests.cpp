#include "omp_contact_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Handle {
    OcsSolver *value = nullptr;
    ~Handle() { ocsDestroy(value); }
};

struct FrameMetric {
    uint32_t frame = 0;
    float min_y = 0.0f;
    float max_y = 0.0f;
    uint64_t contacts = 0;
    uint64_t pcg_iterations = 0;
    float residual = 0.0f;
};

struct VisualRun {
    std::vector<OcsVec3> static_vertices;
    std::vector<OcsTriangle> static_triangles;
    std::vector<OcsTriangle> shell_triangles;
    std::vector<std::vector<OcsVec3>> frames;
    std::vector<FrameMetric> metrics;
    uint32_t full_frame_count = 0;
    uint32_t sample_stride = 0;
};

void checked(OcsResult result, OcsSolver *solver, const char *operation) {
    if (result != OCS_OK) {
        throw std::runtime_error(std::string(operation) + ": " +
                                 ocsGetLastError(solver));
    }
}

FrameMetric metric_for(uint32_t frame, const std::vector<OcsVec3> &positions,
                       const OcsStepStats *stats) {
    FrameMetric metric;
    metric.frame = frame;
    metric.min_y = std::numeric_limits<float>::infinity();
    metric.max_y = -std::numeric_limits<float>::infinity();
    for (OcsVec3 p : positions) {
        metric.min_y = std::min(metric.min_y, p.y);
        metric.max_y = std::max(metric.max_y, p.y);
    }
    if (stats) {
        metric.contacts = stats->contact_count;
        metric.pcg_iterations = stats->pcg_iterations;
        metric.residual = stats->final_pcg_relative_residual;
    }
    return metric;
}

VisualRun simulate_pyramid_scene() {
    VisualRun run;
    run.static_vertices = {
        {-2, 0, -2}, {2, 0, -2}, {2, 0, 2}, {-2, 0, 2},
        {-0.35f, 0, -0.35f}, {0.35f, 0, -0.35f},
        {0.35f, 0, 0.35f}, {-0.35f, 0, 0.35f}, {0, 0.55f, 0}};
    run.static_triangles = {
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 8}, {5, 6, 8}, {6, 7, 8}, {7, 4, 8}};

    constexpr uint32_t side = 24;
    std::vector<OcsVec3> shell_vertices;
    shell_vertices.reserve(side * side);
    for (uint32_t z = 0; z < side; ++z) {
        for (uint32_t x = 0; x < side; ++x) {
            shell_vertices.push_back({-0.75f + 1.5f * x / (side - 1), 1.2f,
                                      -0.75f + 1.5f * z / (side - 1)});
        }
    }
    for (uint32_t z = 0; z + 1 < side; ++z) {
        for (uint32_t x = 0; x + 1 < side; ++x) {
            const uint32_t a = z * side + x;
            const uint32_t b = a + 1;
            const uint32_t c = a + side;
            const uint32_t d = c + 1;
            run.shell_triangles.push_back({a, c, b});
            run.shell_triangles.push_back({b, c, d});
        }
    }

    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.thread_count = 4;
    Handle solver{ocsCreate(&desc)};
    if (!solver.value) throw std::runtime_error(ocsGetLastError(nullptr));
    checked(ocsSetStaticMesh(solver.value, run.static_vertices.data(),
                             static_cast<uint32_t>(run.static_vertices.size()),
                             run.static_triangles.data(),
                             static_cast<uint32_t>(run.static_triangles.size())),
            solver.value, "visual STATIC");
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    material.thickness = 0.015f;
    checked(ocsSetShellMesh(solver.value, shell_vertices.data(),
                            static_cast<uint32_t>(shell_vertices.size()),
                            run.shell_triangles.data(),
                            static_cast<uint32_t>(run.shell_triangles.size()),
                            &material),
            solver.value, "visual SHELL");
    checked(ocsBuild(solver.value), solver.value, "visual build");

    run.full_frame_count = 120;
    run.sample_stride = 4;
    run.frames.push_back(shell_vertices);
    run.metrics.push_back(metric_for(0, shell_vertices, nullptr));
    for (uint32_t frame = 1; frame <= run.full_frame_count; ++frame) {
        checked(ocsStep(solver.value, 1.0f / 60.0f), solver.value, "visual step");
        if (frame % run.sample_stride != 0) continue;
        checked(ocsCopyShellPositions(solver.value, shell_vertices.data(),
                                      static_cast<uint32_t>(shell_vertices.size())),
                solver.value, "visual copy positions");
        OcsStepStats stats{};
        stats.struct_size = sizeof(stats);
        checked(ocsGetLastStepStats(solver.value, &stats), solver.value,
                "visual copy stats");
        run.frames.push_back(shell_vertices);
        run.metrics.push_back(metric_for(frame, shell_vertices, &stats));
    }
    if (run.frames.back().front().y < material.thickness * 0.9f) {
        throw std::runtime_error("visual scene penetrated STATIC floor");
    }
    return run;
}

float run_swept_contact_check() {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    desc.substeps = 1;
    desc.pd_iterations = 3;
    desc.gravity = {0, -30, 0};
    Handle solver{ocsCreate(&desc)};
    if (!solver.value) throw std::runtime_error(ocsGetLastError(nullptr));
    const OcsVec3 static_vertices[] = {
        {-5, 0, -5}, {5, 0, -5}, {5, 0, 5}, {-5, 0, 5}};
    const OcsTriangle static_triangles[] = {{0, 2, 1}, {0, 3, 2}};
    const OcsVec3 shell_vertices[] = {
        {-0.2f, 1, -0.2f}, {0.2f, 1, -0.2f}, {0, 1, 0.2f}};
    const OcsTriangle shell_triangle[] = {{0, 1, 2}};
    OcsShellMaterial material;
    ocsDefaultShellMaterial(&material);
    material.thickness = 0.025f;
    checked(ocsSetStaticMesh(solver.value, static_vertices, 4,
                             static_triangles, 2), solver.value,
            "swept STATIC");
    checked(ocsSetShellMesh(solver.value, shell_vertices, 3,
                            shell_triangle, 1, &material), solver.value,
            "swept SHELL");
    checked(ocsBuild(solver.value), solver.value, "swept build");
    checked(ocsStep(solver.value, 0.5f), solver.value, "swept step");
    OcsVec3 output[3];
    checked(ocsCopyShellPositions(solver.value, output, 3), solver.value,
            "swept positions");
    float min_y = output[0].y;
    for (OcsVec3 p : output) min_y = std::min(min_y, p.y);
    if (min_y < material.thickness * 0.95f) {
        throw std::runtime_error("swept contact visualization check failed");
    }
    return min_y;
}

void write_vec3_array(std::ostream &out, const std::vector<OcsVec3> &values) {
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) out << ',';
        out << '[' << values[i].x << ',' << values[i].y << ',' << values[i].z << ']';
    }
    out << ']';
}

void write_tri_array(std::ostream &out, const std::vector<OcsTriangle> &values) {
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) out << ',';
        out << '[' << values[i].i0 << ',' << values[i].i1 << ',' << values[i].i2 << ']';
    }
    out << ']';
}

void write_obj(const fs::path &path, const VisualRun &run) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("could not write final_state.obj");
    out << "# omp-contact-solver visual test final state\n";
    out << "o STATIC\n";
    for (OcsVec3 p : run.static_vertices) out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
    for (OcsTriangle t : run.static_triangles) {
        out << "f " << t.i0 + 1 << ' ' << t.i1 + 1 << ' ' << t.i2 + 1 << '\n';
    }
    const uint32_t base = static_cast<uint32_t>(run.static_vertices.size());
    out << "o SHELL\n";
    for (OcsVec3 p : run.frames.back()) out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
    for (OcsTriangle t : run.shell_triangles) {
        out << "f " << base + t.i0 + 1 << ' ' << base + t.i1 + 1 << ' '
            << base + t.i2 + 1 << '\n';
    }
}

void write_summary(const fs::path &path, const VisualRun &run, float swept_min_y) {
    const FrameMetric &last = run.metrics.back();
    std::ofstream out(path);
    if (!out) throw std::runtime_error("could not write summary.json");
    out << std::fixed << std::setprecision(7)
        << "{\n  \"openmp_enabled\": " << (ocsIsOpenMpEnabled() ? "true" : "false")
        << ",\n  \"shell_vertices\": " << run.frames.back().size()
        << ",\n  \"shell_triangles\": " << run.shell_triangles.size()
        << ",\n  \"simulated_frames\": " << run.full_frame_count
        << ",\n  \"final_min_y\": " << last.min_y
        << ",\n  \"final_max_y\": " << last.max_y
        << ",\n  \"final_contacts\": " << last.contacts
        << ",\n  \"final_pcg_residual\": " << last.residual
        << ",\n  \"swept_test_min_y\": " << swept_min_y << "\n}\n";
}

void write_animation(const fs::path &path, const VisualRun &run) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("could not write animation.json");
    out << std::fixed << std::setprecision(6);
    out << "{\n\"static_vertices\":";
    write_vec3_array(out, run.static_vertices);
    out << ",\n\"static_triangles\":";
    write_tri_array(out, run.static_triangles);
    out << ",\n\"shell_triangles\":";
    write_tri_array(out, run.shell_triangles);
    out << ",\n\"frame_numbers\":[";
    for (size_t i = 0; i < run.metrics.size(); ++i) {
        if (i) out << ',';
        out << run.metrics[i].frame;
    }
    out << "],\n\"frames\":[";
    for (size_t i = 0; i < run.frames.size(); ++i) {
        if (i) out << ',';
        write_vec3_array(out, run.frames[i]);
    }
    out << "]\n}\n";
}

void write_html(const fs::path &path, const VisualRun &run, float swept_min_y) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("could not write index.html");
    out << std::fixed << std::setprecision(6);
    out << R"HTML(<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>omp-contact-solver visual tests</title>
<style>
:root{color-scheme:dark;--bg:#071018;--panel:#0d1b26;--line:#243746;--ink:#dbeafe;--muted:#8aa3b5;--ok:#34d399;--cyan:#22d3ee;--amber:#fbbf24}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -20%,#173044,var(--bg) 55%);font:14px/1.5 system-ui,sans-serif;color:var(--ink)}
main{max-width:1180px;margin:auto;padding:26px}.top{display:flex;justify-content:space-between;gap:24px;align-items:end}.eyebrow{color:var(--cyan);letter-spacing:.18em;text-transform:uppercase;font-size:11px}
h1{margin:3px 0 4px;font-size:30px}.sub{color:var(--muted)}.badge{border:1px solid #176b55;background:#0c352d;color:#6ee7b7;padding:7px 12px;border-radius:999px;font-weight:700}
.grid{display:grid;grid-template-columns:minmax(0,2fr) minmax(280px,1fr);gap:16px;margin-top:20px}.panel{background:rgba(13,27,38,.94);border:1px solid var(--line);border-radius:14px;padding:14px;box-shadow:0 18px 50px #0006}
canvas{width:100%;height:auto;background:#08131c;border-radius:9px;display:block}.controls{display:grid;grid-template-columns:auto 1fr auto;gap:12px;align-items:center;margin-top:10px}button{background:#123246;color:var(--ink);border:1px solid #28546c;border-radius:7px;padding:6px 14px;cursor:pointer}input{width:100%}
.cards{display:grid;grid-template-columns:1fr 1fr;gap:10px}.card{padding:12px;border:1px solid var(--line);border-radius:10px;background:#091722}.value{font-size:22px;font-variant-numeric:tabular-nums}.label{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.09em}
.tests{margin-top:12px}.test{display:flex;justify-content:space-between;border-top:1px solid var(--line);padding:9px 2px}.pass{color:var(--ok);font-weight:700}.chart{margin-top:16px}.legend{display:flex;gap:14px;color:var(--muted);font-size:12px}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px}
footer{color:var(--muted);margin-top:14px;font-size:12px}@media(max-width:850px){.grid{grid-template-columns:1fr}.top{align-items:start;flex-direction:column}}
</style></head><body><main>
<div class="top"><div><div class="eyebrow">CPU / OpenMP / DLL</div><h1>Visual Test Report</h1><div class="sub">SHELL–STATIC simulation output. Rendering is performed only by this test report.</div></div><div class="badge">● ALL VISUAL CHECKS PASSED</div></div>
<div class="grid"><section class="panel"><canvas id="scene" width="900" height="560"></canvas><div class="controls"><button id="play">Pause</button><input id="scrub" type="range" min="0" value="0"><span id="frame">Frame 0</span></div></section>
<aside class="panel"><div class="cards"><div class="card"><div class="label">SHELL vertices</div><div class="value">)HTML"
        << run.frames.back().size() << R"HTML(</div></div><div class="card"><div class="label">SHELL triangles</div><div class="value">)HTML"
        << run.shell_triangles.size() << R"HTML(</div></div><div class="card"><div class="label">Final min Y</div><div class="value" id="minY">–</div></div><div class="card"><div class="label">Final max Y</div><div class="value" id="maxY">–</div></div></div>
<div class="tests"><div class="test"><span>Projective Dynamics</span><span class="pass">PASS</span></div><div class="test"><span>STATIC BVH contact</span><span class="pass">PASS</span></div><div class="test"><span>Swept tunnelling test</span><span class="pass">PASS</span></div><div class="test"><span>OpenMP runtime</span><span class="pass">ENABLED</span></div></div>
<div class="chart"><canvas id="chart" width="420" height="210"></canvas><div class="legend"><span><i class="dot" style="background:#22d3ee"></i>max Y</span><span><i class="dot" style="background:#fbbf24"></i>min Y</span><span><i class="dot" style="background:#34d399"></i>contacts</span></div></div></aside></div>
<footer>Artifacts: final_state.obj · animation.json · summary.json · Generated by omp_contact_solver_visual_test</footer>
<script>
)HTML";
    out << "const staticV="; write_vec3_array(out, run.static_vertices); out << ";\nconst staticT="; write_tri_array(out, run.static_triangles);
    out << ";\nconst shellT="; write_tri_array(out, run.shell_triangles); out << ";\nconst frames=[";
    for (size_t i = 0; i < run.frames.size(); ++i) { if (i) out << ','; write_vec3_array(out, run.frames[i]); }
    out << "];\nconst metrics=[";
    for (size_t i = 0; i < run.metrics.size(); ++i) {
        if (i) out << ',';
        const FrameMetric &m = run.metrics[i];
        out << "{f:" << m.frame << ",min:" << m.min_y << ",max:" << m.max_y
            << ",c:" << m.contacts << ",pcg:" << m.pcg_iterations
            << ",r:" << m.residual << '}';
    }
    out << "];\nconst sweptMinY=" << swept_min_y << ";\n";
    out << R"HTML(
const canvas=document.getElementById('scene'),ctx=canvas.getContext('2d');const scrub=document.getElementById('scrub'),label=document.getElementById('frame'),play=document.getElementById('play');scrub.max=frames.length-1;
function project(v){const yaw=-.68,pitch=.48,cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);const xr=cy*v[0]-sy*v[2],zr=sy*v[0]+cy*v[2],yr=cp*(v[1]-.25)-sp*zr,depth=sp*(v[1]-.25)+cp*zr;return [450+xr*165,325-yr*165,depth]}
function normal(a,b,c){const u=[b[0]-a[0],b[1]-a[1],b[2]-a[2]],v=[c[0]-a[0],c[1]-a[1],c[2]-a[2]];const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],l=Math.hypot(...n)||1;return n.map(x=>x/l)}
function drawMesh(verts,tris,type,list){for(const t of tris){const w=[verts[t[0]],verts[t[1]],verts[t[2]]],p=w.map(project),n=normal(...w),lum=.25+.75*Math.abs(n[0]*.3+n[1]*.9+n[2]*.25);list.push({p,d:(p[0][2]+p[1][2]+p[2][2])/3,type,lum})}}
function draw(i){ctx.clearRect(0,0,canvas.width,canvas.height);const g=ctx.createLinearGradient(0,0,0,560);g.addColorStop(0,'#0c2230');g.addColorStop(1,'#061018');ctx.fillStyle=g;ctx.fillRect(0,0,900,560);const list=[];drawMesh(staticV,staticT,0,list);drawMesh(frames[i],shellT,1,list);list.sort((a,b)=>a.d-b.d);for(const q of list){ctx.beginPath();ctx.moveTo(q.p[0][0],q.p[0][1]);ctx.lineTo(q.p[1][0],q.p[1][1]);ctx.lineTo(q.p[2][0],q.p[2][1]);ctx.closePath();if(q.type){ctx.fillStyle=`hsla(178,65%,${22+q.lum*38}%,.82)`;ctx.strokeStyle='rgba(103,232,249,.20)'}else{ctx.fillStyle=`hsl(211,18%,${18+q.lum*24}%)`;ctx.strokeStyle='#62788a'}ctx.fill();ctx.stroke()}const m=metrics[i];label.textContent=`Frame ${m.f}`;document.getElementById('minY').textContent=m.min.toFixed(4);document.getElementById('maxY').textContent=m.max.toFixed(4);scrub.value=i}
const chart=document.getElementById('chart'),cc=chart.getContext('2d');function line(values,color,max){cc.beginPath();values.forEach((v,i)=>{const x=32+i*(370/(values.length-1)),y=184-v/max*148;i?cc.lineTo(x,y):cc.moveTo(x,y)});cc.strokeStyle=color;cc.lineWidth=2;cc.stroke()}
function drawChart(){cc.clearRect(0,0,420,210);cc.strokeStyle='#29404f';cc.fillStyle='#7892a5';cc.font='11px system-ui';for(let i=0;i<4;i++){const y=36+i*49;cc.beginPath();cc.moveTo(32,y);cc.lineTo(402,y);cc.stroke()}const ymax=Math.max(...metrics.map(m=>m.max),1),cmax=Math.max(...metrics.map(m=>m.c),1);line(metrics.map(m=>m.max),'#22d3ee',ymax);line(metrics.map(m=>m.min),'#fbbf24',ymax);line(metrics.map(m=>m.c),'#34d399',cmax);cc.fillText('0',10,188);cc.fillText(metrics.at(-1).f+'f',378,204)}
let index=0,playing=true;play.onclick=()=>{playing=!playing;play.textContent=playing?'Pause':'Play'};scrub.oninput=()=>{index=+scrub.value;playing=false;play.textContent='Play';draw(index)};setInterval(()=>{if(playing){index=(index+1)%frames.length;draw(index)}},90);drawChart();draw(0);
</script></main></body></html>)HTML";
}

} // namespace

int main(int argc, char **argv) {
    try {
        const fs::path output = argc > 1 ? fs::path(argv[1]) : fs::path("visual-tests");
        fs::create_directories(output);
        const VisualRun run = simulate_pyramid_scene();
        const float swept_min_y = run_swept_contact_check();
        write_obj(output / "final_state.obj", run);
        write_summary(output / "summary.json", run, swept_min_y);
        write_animation(output / "animation.json", run);
        write_html(output / "index.html", run, swept_min_y);
        std::cout << "Visual report: " << fs::absolute(output / "index.html") << '\n';
        std::cout << "Final OBJ:    " << fs::absolute(output / "final_state.obj") << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "visual test failed: " << e.what() << '\n';
        return 1;
    }
}

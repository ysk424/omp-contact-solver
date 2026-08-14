# omp-contact-solver

A compact CPU cloth simulation DLL using C++17 and OpenMP. It is a new,
standalone solver and does not depend on CUDA or `ppf-contact-solver`.

The library computes simulation vertex positions only. Rendering remains the
responsibility of the caller.

## Scope

Supported:

- one deformable triangle `SHELL` mesh;
- one topology-stable, animated triangle `STATIC` mesh (it may contain disconnected objects);
- gravity, inertia, damping, stretch and simple bending;
- high-strength seam threads between disconnected SHELL parts;
- optional per-triangle principal Strain Limit coupled through PD/ADMM;
- two-sided `SHELL` vertex versus `STATIC` triangle contact;
- thickness, friction and restitution;
- OpenMP parallel prediction, constraint projection, PCG and BVH queries;
- a C ABI exported from a Windows DLL or Unix shared library.

Intentionally unsupported:

- PIN constraints;
- SHELL self-collision or SHELL-SHELL collision;
- topology-changing STATIC modifiers;
- edge-edge collision and exact cloth CCD;
- tets, rods, sand, rigid bodies, plasticity and rendering.

The elastic step is Projective Dynamics. Unique triangle edges provide stretch
constraints; the two opposite vertices of each interior edge provide the
compact bending constraint. The global system is solved by a symmetric Gauss-
Seidel-preconditioned, OpenMP-parallel matrix-free PCG. STATIC triangles are
stored in a median-split BVH whose bounds are refitted with OpenMP for animated
vertices. Swept vertex-triangle and closest-point
queries supply finite contact targets to the same global solve. Optional hard
contact safety passes are available, while the Blender Extension defaults them
to zero so they cannot invalidate the coupled Strain Limit. Seam threads use their
captured rest length, finite stiffness, and the same Projective Dynamics
local/global solve as stretch and contact. There is no post-solve seam
projection.

The Strain Limit projects each triangle deformation gradient onto a maximum
singular-value bound (5% means `sigma_max <= 1.05`). Scaled ADMM dual updates
feed the projection back into the same matrix-free global system. It is not a
post-solve vertex clamp. `strain_limit_stiffness` is the ADMM penalty controlling
convergence rather than a physical Young's modulus.

## Build

OpenMP is required; configuration fails when it is unavailable.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

With MinGW on Windows the main artifacts are:

```text
build/omp_contact_solver.dll
build/libomp_contact_solver.dll.a
build/drop_cloth.exe
```

`drop_cloth` advances a 576-vertex SHELL over a STATIC floor and pyramid, then
prints the simulated height range and solver statistics. It performs no
rendering.

## Visual test report

The solver DLL remains rendering-free. A separate test executable records its
vertex output and generates a self-contained HTML canvas report:

```powershell
cmake --build build --target visual-test-report
start build/visual-tests/index.html
```

The report contains a playable cloth animation, a strain-limit stress check,
and charts for height, contact projections and PCG residual.
`build/visual-tests/final_state.obj` contains the
same final STATIC and SHELL meshes for inspection in Blender. CTest also runs
the report generator and treats generation or simulation failure as a failed
test. `animation.json` contains all sampled vertex frames for DCC import.

MSVC is also supported. CMake links the compiler's OpenMP runtime into the DLL
dependency set.

The MinGW build statically includes its GCC, C++, pthread and OpenMP runtimes,
so `omp_contact_solver.dll` is the only non-system runtime file to distribute.
An MSVC build uses the OpenMP/runtime components supplied with that toolchain.

## Blender Extension

The Windows x64 Blender Extension wraps the C ABI with `ctypes`. It assigns one
Blender mesh as `SHELL`, evaluates one animated mesh as `STATIC`, runs the CPU
solver, and bakes the result as absolute Shape Keys. Rendering remains entirely
in Blender; the DLL still has no rendering or GPU dependency.

Configure with a Blender 4.2 or newer executable, then build and test the
installable package:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release `
  -DOCS_BLENDER_EXECUTABLE="C:/path/to/blender.exe"
cmake --build build --target blender-extension
cmake --build build --target blender-extension-test
```

Install `build/packages/omp_contact_solver-0.5.0-windows-x64.zip` from
**Edit > Preferences > Extensions > Install from Disk**. The controls are in
**3D View > Sidebar > OMP Cloth**. Assign source meshes, then use **Prepare
Simulation Copies**. The Extension evaluates the source `SHELL` at the first
bake frame and copies the source `STATIC` object with its Armature, Mesh Cache,
and other animation modifiers into a separate `OMP Contact Simulation`
collection. At every frame it evaluates that visible collision copy and sends
its vertices to the DLL. By default a topology-stable final Mask keeps body
polygons crossing world Z=0.40 through Z=1.45; it leaves the source Mesh Cache
and Armature inputs intact and requires no caps because contact is two-sided.
Tiny triangles below the native solver's area
tolerance are omitted from collision without changing the visible mesh. It
also pairs nearby boundary vertices from
disconnected SHELL parts as finite high-strength seam threads without merging
the mesh. Baking and clearing then operate only on the prepared `SHELL`; source
meshes and their existing Shape Keys are not overwritten.

## DLL API

Include [`include/omp_contact_solver.h`](include/omp_contact_solver.h). The
public boundary uses an opaque handle, fixed-width integers and plain C
structures; no C++ or STL type crosses the DLL boundary.

Typical call order:

```c
OcsSolverDesc desc;
ocsDefaultSolverDesc(&desc);
OcsSolver *solver = ocsCreate(&desc);

ocsSetStaticMesh(solver, static_vertices, static_vertex_count,
                 static_triangles, static_triangle_count);

OcsShellMaterial material;
ocsDefaultShellMaterial(&material);
ocsSetShellMesh(solver, shell_vertices, shell_vertex_count,
                shell_triangles, shell_triangle_count, &material);
ocsSetShellSeams(solver, seams, seam_count); /* Optional. */

ocsBuild(solver);
ocsUpdateStaticVertices(solver, animated_static_vertices,
                        static_vertex_count); /* Optional per frame. */
ocsStep(solver, 1.0f / 60.0f);
ocsCopyShellPositions(solver, output, shell_vertex_count);
ocsDestroy(solver);
```

`ocsBuild()` freezes both topologies, creates the cloth constraints and builds
the STATIC BVH. `ocsUpdateStaticVertices()` queues a deformation target for the
next step; the DLL interpolates it over the configured substeps and refits the
BVH without rebuilding its topology. Create separate solver handles for
independent simulations.
Calls on one handle must not overlap; different handles may be advanced by
different host threads. `thread_count = 0` uses `omp_get_max_threads()`.

## Numerical limitations

Contact is vertex-triangle only. The swept test greatly reduces vertex
tunnelling, but an edge can still pass through a thin triangle without a vertex
crossing it. Animated STATIC motion is sampled at each solver substep; use more
substeps for fast or very thin motion. STATIC modifiers must preserve vertex
count and topology. Because no PIN is
supported, all SHELL vertices are dynamic.

The compact bending constraint is not parameter-compatible with the CUDA PPF
solver. Treat stiffness values as parameters of this solver and calibrate them
for the mesh resolution in use.

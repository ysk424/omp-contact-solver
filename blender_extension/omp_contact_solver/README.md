# OMP Contact Solver for Blender

This Windows x64 Blender Extension runs the bundled CPU/OpenMP solver and
bakes its output as absolute Shape Keys. It does not render and does not use a
GPU.

1. Install the generated ZIP with **Edit > Preferences > Extensions > Install
   from Disk**.
2. Open **3D View > Sidebar > OMP Cloth**.
3. Assign one triangle mesh as `SHELL` and another as `STATIC`.
4. Choose the frame range and material/solver values, then use **Bake
   Simulation**.

`STATIC` is evaluated once at the start frame and remains immutable during the
bake. `SHELL` uses its original mesh topology. Existing Shape Keys are left
untouched: make a mesh copy without Shape Keys before baking. **Clear Bake**
only removes a Shape Key datablock created and owned by this Extension.

The current solver intentionally has no pins, self-collision, moving STATIC
geometry, GPU backend, or renderer.

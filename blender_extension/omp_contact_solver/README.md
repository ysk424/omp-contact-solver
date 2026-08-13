# OMP Contact Solver for Blender

This Windows x64 Blender Extension runs the bundled CPU/OpenMP solver and
bakes its output as absolute Shape Keys. It does not render and does not use a
GPU.

1. Install the generated ZIP with **Edit > Preferences > Extensions > Install
   from Disk**.
2. Open **3D View > Sidebar > OMP Cloth**.
3. Assign a garment as source `SHELL` and a body/collider as source `STATIC`.
4. Choose the start frame, then use **Prepare Simulation Copies**. This creates
   evaluated `SHELL` and `STATIC` snapshots in a separate collection, applies
   source modifiers, and removes tiny `STATIC` triangles rejected by the DLL.
5. Choose the frame range and material/solver values, then use **Bake
   Simulation**. The bake is written only to the prepared `SHELL`.

Both sources are evaluated once at the start frame and remain immutable during
the bake. This captures Armature, Mesh Cache, and other evaluated modifiers in
the prepared copies. Existing source Shape Keys are left untouched. **Clear
Bake** only removes a Shape Key datablock created and owned by this Extension;
**Clear Prepared** removes the generated collection objects and restores source
`SHELL` visibility.

The current solver intentionally has no pins, self-collision, moving STATIC
geometry, GPU backend, or renderer.

# OMP Contact Solver for Blender

This Windows x64 Blender Extension runs the bundled CPU/OpenMP solver and
bakes its output as absolute Shape Keys. It does not render and does not use a
GPU.

1. Install the generated ZIP with **Edit > Preferences > Extensions > Install
   from Disk**.
2. Open **3D View > Sidebar > OMP Cloth**.
3. Assign a garment as source `SHELL` and a body/collider as source `STATIC`.
4. Choose the start frame, then use **Prepare Simulation Copies**. This creates
   an evaluated `SHELL` snapshot and a visible `STATIC` object copy in a separate
   collection. The `STATIC` copy retains Armature, Mesh Cache, and other source
   animation modifiers. A topology-stable Mask crops its evaluated collision
   body between the configured world-Z planes (40–145 cm by default). Open cut
   boundaries need no caps because collision is two-sided. Tiny collision
   triangles are skipped, and nearby boundaries between disconnected `SHELL` parts
   are detected as seam threads.
5. Choose the frame range and material/solver values, then use **Bake
   Simulation**. The bake is written only to the prepared `SHELL`.

The prepared `STATIC` is evaluated every frame and its deformation is sent to
the DLL, which interpolates and refits the collision BVH over its substeps.
Its modifiers must preserve topology. Existing source Shape Keys are left
untouched. **Clear
Bake** only removes a Shape Key datablock created and owned by this Extension;
**Clear Prepared** removes the generated collection objects and restores source
`SHELL` visibility.

**Auto Seam Threads** gives each detected pair a finite Projective Dynamics
constraint at its initial distance without merging vertices, so UVs and
materials remain intact. **Seam Distance** controls pair detection and requires
running Prepare again after a change. The default thread stiffness is higher
than ordinary cloth Stretch, but it is solved together with cloth and contact;
there is no forced seam correction after contact.

**Strain Limit** bounds each triangle's tensile principal stretch (5% by
default) with a PD/ADMM constraint in the global solve. It is not a post-bake
edge clamp. The default 2 mm Thickness avoids making the limiter fight an
artificially inflated initial contact band. **Collision Safety Passes** defaults
to zero because a hard projection after the global solve can break the strain
bound; nonzero values trade strain consistency for strict final separation.

The current solver intentionally has no pins, self-collision, topology-changing
STATIC animation, GPU backend, or renderer.

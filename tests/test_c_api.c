#include "omp_contact_solver.h"

#include <stdio.h>

int main(void) {
    OcsSolverDesc desc;
    ocsDefaultSolverDesc(&desc);
    if (desc.struct_size != sizeof(desc) || ocsGetAbiVersion() != OCS_ABI_VERSION ||
        !ocsIsOpenMpEnabled()) {
        fputs("C ABI metadata check failed\n", stderr);
        return 1;
    }
    OcsSolver *solver = ocsCreate(&desc);
    if (!solver) {
        fprintf(stderr, "C ABI create failed: %s\n", ocsGetLastError(NULL));
        return 1;
    }
    if (ocsSetShellSeams(solver, NULL, 0) != OCS_OK) {
        fprintf(stderr, "C ABI seam clear failed: %s\n", ocsGetLastError(solver));
        ocsDestroy(solver);
        return 1;
    }
    {
        const OcsVec3 vertex = {0.0f, 0.0f, 0.0f};
        if (ocsUpdateStaticVertices(solver, &vertex, 1) !=
            OCS_ERROR_INVALID_STATE) {
            fputs("C ABI animated STATIC state check failed\n", stderr);
            ocsDestroy(solver);
            return 1;
        }
    }
    ocsDestroy(solver);
    puts("C ABI smoke test passed.");
    return 0;
}

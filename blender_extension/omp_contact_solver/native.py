"""Small ctypes binding for the omp-contact-solver C ABI.

This module deliberately has no bpy dependency so the DLL boundary can also be
smoke-tested with a normal Python interpreter.
"""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path
from typing import Iterable, Sequence


OCS_ABI_VERSION = 1
OCS_OK = 0


class NativeSolverError(RuntimeError):
    """Raised when the native solver rejects an operation."""


class Vec3(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Triangle(ctypes.Structure):
    _fields_ = [
        ("i0", ctypes.c_uint32),
        ("i1", ctypes.c_uint32),
        ("i2", ctypes.c_uint32),
    ]


class SolverDesc(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("gravity", Vec3),
        ("substeps", ctypes.c_uint32),
        ("pd_iterations", ctypes.c_uint32),
        ("pcg_iterations", ctypes.c_uint32),
        ("pcg_relative_tolerance", ctypes.c_float),
        ("collision_iterations", ctypes.c_uint32),
        ("velocity_damping", ctypes.c_float),
        ("thread_count", ctypes.c_uint32),
    ]


class ShellMaterial(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("density", ctypes.c_float),
        ("stretch_stiffness", ctypes.c_float),
        ("bend_stiffness", ctypes.c_float),
        ("thickness", ctypes.c_float),
        ("friction", ctypes.c_float),
        ("restitution", ctypes.c_float),
    ]


class StepStats(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("substeps", ctypes.c_uint32),
        ("pd_iterations", ctypes.c_uint32),
        ("pcg_iterations", ctypes.c_uint64),
        ("contact_count", ctypes.c_uint64),
        ("final_pcg_relative_residual", ctypes.c_float),
    ]


def bundled_library_path() -> Path:
    """Return the platform-specific library path inside the Extension."""
    if sys.platform == "win32":
        filename = "omp_contact_solver.dll"
    elif sys.platform == "darwin":
        filename = "libomp_contact_solver.dylib"
    else:
        filename = "libomp_contact_solver.so"
    return Path(__file__).resolve().parent / "bin" / filename


def _as_vec3_array(values: Iterable[Sequence[float]]):
    converted = [Vec3(float(value[0]), float(value[1]), float(value[2])) for value in values]
    return (Vec3 * len(converted))(*converted)


def _as_triangle_array(values: Iterable[Sequence[int]]):
    converted = [
        Triangle(int(value[0]), int(value[1]), int(value[2])) for value in values
    ]
    return (Triangle * len(converted))(*converted)


class SolverLibrary:
    """Loaded DLL and fully declared C function table."""

    def __init__(self, path: os.PathLike[str] | str | None = None):
        self.path = Path(path) if path is not None else bundled_library_path()
        if not self.path.is_file():
            raise NativeSolverError(f"Solver library was not found: {self.path}")

        dll_directory = None
        try:
            if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
                dll_directory = os.add_dll_directory(str(self.path.parent))
            self.api = ctypes.CDLL(str(self.path))
        except OSError as exc:
            raise NativeSolverError(f"Could not load solver library {self.path}: {exc}") from exc
        finally:
            if dll_directory is not None:
                dll_directory.close()

        self._declare_functions()
        abi = int(self.api.ocsGetAbiVersion())
        if abi != OCS_ABI_VERSION:
            raise NativeSolverError(
                f"Solver ABI mismatch: Extension expects {OCS_ABI_VERSION}, DLL reports {abi}"
            )

    def _declare_functions(self) -> None:
        api = self.api
        api.ocsGetAbiVersion.argtypes = []
        api.ocsGetAbiVersion.restype = ctypes.c_uint32
        api.ocsIsOpenMpEnabled.argtypes = []
        api.ocsIsOpenMpEnabled.restype = ctypes.c_int32
        api.ocsDefaultSolverDesc.argtypes = [ctypes.POINTER(SolverDesc)]
        api.ocsDefaultSolverDesc.restype = None
        api.ocsDefaultShellMaterial.argtypes = [ctypes.POINTER(ShellMaterial)]
        api.ocsDefaultShellMaterial.restype = None
        api.ocsCreate.argtypes = [ctypes.POINTER(SolverDesc)]
        api.ocsCreate.restype = ctypes.c_void_p
        api.ocsDestroy.argtypes = [ctypes.c_void_p]
        api.ocsDestroy.restype = None
        api.ocsSetStaticMesh.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(Vec3),
            ctypes.c_uint32,
            ctypes.POINTER(Triangle),
            ctypes.c_uint32,
        ]
        api.ocsSetStaticMesh.restype = ctypes.c_int32
        api.ocsSetShellMesh.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(Vec3),
            ctypes.c_uint32,
            ctypes.POINTER(Triangle),
            ctypes.c_uint32,
            ctypes.POINTER(ShellMaterial),
        ]
        api.ocsSetShellMesh.restype = ctypes.c_int32
        api.ocsBuild.argtypes = [ctypes.c_void_p]
        api.ocsBuild.restype = ctypes.c_int32
        api.ocsStep.argtypes = [ctypes.c_void_p, ctypes.c_float]
        api.ocsStep.restype = ctypes.c_int32
        api.ocsGetShellVertexCount.argtypes = [ctypes.c_void_p]
        api.ocsGetShellVertexCount.restype = ctypes.c_uint32
        api.ocsCopyShellPositions.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(Vec3),
            ctypes.c_uint32,
        ]
        api.ocsCopyShellPositions.restype = ctypes.c_int32
        api.ocsGetLastStepStats.argtypes = [ctypes.c_void_p, ctypes.POINTER(StepStats)]
        api.ocsGetLastStepStats.restype = ctypes.c_int32
        api.ocsGetLastError.argtypes = [ctypes.c_void_p]
        api.ocsGetLastError.restype = ctypes.c_char_p

    @property
    def openmp_enabled(self) -> bool:
        return bool(self.api.ocsIsOpenMpEnabled())

    def default_desc(self) -> SolverDesc:
        value = SolverDesc()
        self.api.ocsDefaultSolverDesc(ctypes.byref(value))
        return value

    def default_material(self) -> ShellMaterial:
        value = ShellMaterial()
        self.api.ocsDefaultShellMaterial(ctypes.byref(value))
        return value

    def create(self, desc: SolverDesc) -> "Solver":
        return Solver(self, desc)


class Solver:
    """RAII-style wrapper around one opaque OcsSolver handle."""

    def __init__(self, library: SolverLibrary, desc: SolverDesc):
        self.library = library
        self.handle = library.api.ocsCreate(ctypes.byref(desc))
        if not self.handle:
            raise NativeSolverError("Could not create the native solver")

    def close(self) -> None:
        if self.handle:
            self.library.api.ocsDestroy(self.handle)
            self.handle = None

    def __enter__(self) -> "Solver":
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()

    def _error_text(self) -> str:
        raw = self.library.api.ocsGetLastError(self.handle)
        return raw.decode("utf-8", errors="replace") if raw else "unknown native error"

    def _check(self, result: int, operation: str) -> None:
        if int(result) != OCS_OK:
            raise NativeSolverError(
                f"{operation} failed with result {int(result)}: {self._error_text()}"
            )

    def set_static_mesh(self, vertices, triangles) -> None:
        vertex_array = _as_vec3_array(vertices)
        triangle_array = _as_triangle_array(triangles)
        result = self.library.api.ocsSetStaticMesh(
            self.handle,
            vertex_array if len(vertex_array) else None,
            len(vertex_array),
            triangle_array if len(triangle_array) else None,
            len(triangle_array),
        )
        self._check(result, "Setting STATIC mesh")

    def set_shell_mesh(self, vertices, triangles, material: ShellMaterial) -> None:
        vertex_array = _as_vec3_array(vertices)
        triangle_array = _as_triangle_array(triangles)
        result = self.library.api.ocsSetShellMesh(
            self.handle,
            vertex_array,
            len(vertex_array),
            triangle_array,
            len(triangle_array),
            ctypes.byref(material),
        )
        self._check(result, "Setting SHELL mesh")

    def build(self) -> None:
        self._check(self.library.api.ocsBuild(self.handle), "Building solver")

    def step(self, frame_dt: float) -> None:
        self._check(
            self.library.api.ocsStep(self.handle, ctypes.c_float(frame_dt)),
            "Stepping solver",
        )

    def positions(self) -> list[tuple[float, float, float]]:
        count = int(self.library.api.ocsGetShellVertexCount(self.handle))
        values = (Vec3 * count)()
        self._check(
            self.library.api.ocsCopyShellPositions(self.handle, values, count),
            "Copying SHELL positions",
        )
        return [(float(value.x), float(value.y), float(value.z)) for value in values]

    def stats(self) -> StepStats:
        value = StepStats()
        value.struct_size = ctypes.sizeof(StepStats)
        self._check(
            self.library.api.ocsGetLastStepStats(self.handle, ctypes.byref(value)),
            "Reading solver statistics",
        )
        return value


_library: SolverLibrary | None = None


def get_library() -> SolverLibrary:
    global _library
    if _library is None:
        _library = SolverLibrary()
    return _library

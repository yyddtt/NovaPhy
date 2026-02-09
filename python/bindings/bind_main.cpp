#include <pybind11/pybind11.h>

#include "novaphy/novaphy.h"

namespace py = pybind11;

// Forward declarations for sub-module binders
void bind_math(py::module_& m);

PYBIND11_MODULE(_core, m) {
    m.doc() = "NovaPhy: A 3D physics engine for embodied intelligence";

    m.def("version", &novaphy::version, "Returns the NovaPhy version string");

    // Math types and spatial algebra
    bind_math(m);
}

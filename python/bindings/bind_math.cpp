#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "novaphy/math/math_types.h"
#include "novaphy/math/math_utils.h"
#include "novaphy/math/spatial.h"

namespace py = pybind11;
using namespace novaphy;

void bind_math(py::module_& m) {
    // --- Transform ---
    py::class_<Transform>(m, "Transform")
        .def(py::init<>())
        .def(py::init([](const Vec3f& pos, const Vec4f& rot) {
            // rot = [x, y, z, w] (Eigen convention for coeffs())
            return Transform(pos, Quatf(rot[3], rot[0], rot[1], rot[2]));
        }), py::arg("position"), py::arg("rotation"),
            "Create Transform. rotation is [x, y, z, w] quaternion.")
        .def_readwrite("position", &Transform::position)
        .def_property("rotation",
            [](const Transform& t) -> Vec4f {
                // Return as [x, y, z, w]
                return Vec4f(t.rotation.x(), t.rotation.y(),
                             t.rotation.z(), t.rotation.w());
            },
            [](Transform& t, const Vec4f& q) {
                t.rotation = Quatf(q[3], q[0], q[1], q[2]);
            },
            "Quaternion as [x, y, z, w]")
        .def("transform_point", &Transform::transform_point, py::arg("point"))
        .def("transform_vector", &Transform::transform_vector, py::arg("vector"))
        .def("inverse", &Transform::inverse)
        .def("rotation_matrix", &Transform::rotation_matrix)
        .def("__mul__", &Transform::operator*, py::arg("other"))
        .def_static("identity", &Transform::identity)
        .def_static("from_translation", &Transform::from_translation, py::arg("t"))
        .def_static("from_rotation", [](const Vec4f& q) {
            return Transform::from_rotation(Quatf(q[3], q[0], q[1], q[2]));
        }, py::arg("q"), "Create rotation-only Transform. q is [x, y, z, w].")
        .def_static("from_axis_angle", &Transform::from_axis_angle,
                     py::arg("axis"), py::arg("angle"))
        .def("__repr__", [](const Transform& t) {
            return "Transform(pos=[" + std::to_string(t.position.x()) + ", " +
                   std::to_string(t.position.y()) + ", " +
                   std::to_string(t.position.z()) + "], rot=[" +
                   std::to_string(t.rotation.w()) + ", " +
                   std::to_string(t.rotation.x()) + ", " +
                   std::to_string(t.rotation.y()) + ", " +
                   std::to_string(t.rotation.z()) + "])";
        });

    // --- SpatialTransform ---
    py::class_<SpatialTransform>(m, "SpatialTransform")
        .def(py::init<>())
        .def(py::init<const Mat3f&, const Vec3f&>(), py::arg("E"), py::arg("r"))
        .def_readwrite("E", &SpatialTransform::E)
        .def_readwrite("r", &SpatialTransform::r)
        .def("apply_motion", &SpatialTransform::apply_motion, py::arg("v"))
        .def("apply_force", &SpatialTransform::apply_force, py::arg("f"))
        .def("to_matrix", &SpatialTransform::to_matrix)
        .def("inverse", &SpatialTransform::inverse)
        .def("__mul__", &SpatialTransform::operator*, py::arg("other"))
        .def_static("from_transform", &SpatialTransform::from_transform, py::arg("t"))
        .def_static("identity", &SpatialTransform::identity);

    // --- Free functions ---
    m.def("skew", &skew, py::arg("v"), "Skew-symmetric matrix from vector");
    m.def("spatial_cross_motion", &spatial_cross_motion, py::arg("v"), py::arg("u"),
          "Spatial cross product for motion vectors");
    m.def("spatial_cross_force", &spatial_cross_force, py::arg("v"), py::arg("f"),
          "Spatial cross product for force vectors");
    m.def("spatial_inertia_matrix", &spatial_inertia_matrix,
          py::arg("mass"), py::arg("com"), py::arg("I_rot"),
          "Build 6x6 spatial inertia matrix");
    m.def("deg2rad", &deg2rad, py::arg("deg"));
    m.def("rad2deg", &rad2deg, py::arg("rad"));
}

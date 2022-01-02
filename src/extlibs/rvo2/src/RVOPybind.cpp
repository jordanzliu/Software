#include <pybind11/pybind11.h>

#include "extlibs/rvo2/src/RVO.h"
#include "extlibs/rvo2/src/Vector2.h"

namespace py = pybind11;

PYBIND11_MODULE(RVOPybind, m)
{
    py::class_<RVO::Vector2>(m, "Vector2")
        .def(py::init<float, float>())
        .def("x", &RVO::Vector2::x)
        .def("y", &RVO::Vector2::y);
    py::class_<RVO::RVOSimulator>(m, "RVOSimulator")
        .def(py::init<float, float, size_t, float, float, float, float, RVO::Vector2>(),
             py::arg("timeStep"), py::arg("neighborDist"), py::arg("maxNeighbors"),
             py::arg("timeHorizon"), py::arg("timeHorizonObst"), py::arg("radius"),
             py::arg("maxSpeed"), py::arg("velocity"))
        .def("addAgent",
             pybind11::overload_cast<const RVO::Vector2&>(&RVO::RVOSimulator::addAgent))
        .def(
            "getAgentORCALines",
            [](const RVO::RVOSimulator& rvoSimulator, size_t agentNo) {
                const auto numLines = rvoSimulator.getAgentNumORCALines(agentNo);
                py::list result;
                for (auto i = 0; i < numLines; i++)
                {
                    const auto line = rvoSimulator.getAgentORCALine(agentNo, i);
                    py::dict line_dict;
                    line_dict["point"]     = line.point;
                    line_dict["direction"] = line.direction;
                    result.append(line_dict);
                }
                return result;
            },
            py::arg("agentNo"))
        .def("getAgentPrefVelocity", &RVO::RVOSimulator::getAgentPrefVelocity)
        .def("setTimeStep", &RVO::RVOSimulator::setTimeStep)
        .def("doStep", &RVO::RVOSimulator::doStep);
}

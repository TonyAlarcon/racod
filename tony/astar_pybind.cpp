// astar_pybind.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>           // to auto‑convert std::vector/std::pair to Python
#include <chrono>

// pull in your entire implementation in one shot:
#include "astar.cpp"               

namespace py = pybind11;

// wrap your A* dispatcher (same signature as your main’s switch)
PYBIND11_MODULE(astar_ext, m) {
    m.doc() = "PyBind11 bindings for your astar.cpp";

    m.def("run_astar", [](const std::string &map_path,
                          std::pair<int,int> start,
                          std::pair<int,int> goal,
                          int radius,
                          size_t threads,
                          int max_depth,
                          const std::string &algo) {

        // drop the GIL while we do all our C++ work
        py::gil_scoped_release release;
        // load map
        auto grid = loadMap(map_path);

        // pick and run the chosen variant
        auto t0 = std::chrono::high_resolution_clock::now();
        std::vector<std::pair<int,int>> path;
        if      (algo=="serial")         path = astarSerial(grid, start, goal, radius);
        else if (algo=="createjoin")     path = astarCreateJoin(grid, start, goal, radius, threads);
        else if (algo=="pool")           path = astarPersistentThreadPool(grid, start, goal, radius, threads);
        else if (algo=="createjoin_ras") path = astarCreateJoinRAS(grid, start, goal, radius, threads, max_depth);
        else if (algo=="pool_ras")       path = astarPoolRAS(grid, start, goal, radius, threads, max_depth);
        else throw std::runtime_error("Unknown algorithm: "+algo);
        auto t1 = std::chrono::high_resolution_clock::now();

        double ms = std::chrono::duration<double,std::milli>(t1-t0).count();

        py::gil_scoped_acquire acquire;
        return py::make_tuple(path, ms);
    },
    py::arg("map_path"),
    py::arg("start"),
    py::arg("goal"),
    py::arg("radius")    = 1,
    py::arg("threads")   = std::thread::hardware_concurrency(),
    py::arg("max_depth") = 8,
    py::arg("algo")      = "pool_ras");
}

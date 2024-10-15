project(ly.simulator)

include_directories(
    ../../utils/Headers
    ../driver/Headers
)

AddCXXModule(
    NAME monitor
    
    LINKS
        ${OpenCV_LIBS} 
        Threads::Threads
        Eigen3::Eigen
        Python::Python
        pybind11::pybind11
)

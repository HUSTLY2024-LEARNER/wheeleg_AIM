project(ly.solver)

include_directories(
    ../detector/Headers
    ../../utils/Headers
    ../driver/Headers
    ../tracker/Headers
)

AddCXXModule(
    NAME driver
    
    LINKS
        ${OpenCV_LIBS} 
        Eigen3::Eigen
        Threads::Threads
        libgxiapi.so
        Python::Python
        pybind11::pybind11
)

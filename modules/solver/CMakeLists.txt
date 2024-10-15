project(ly.solver)

include_directories(
    ../detector/Headers
    ../../utils/Headers
    ../driver/Headers
    ../tracker/Headers
)

AddCXXModule(
    NAME solver
    
    LINKS
        ${OpenCV_LIBS} 
        Sophus::Sophus
        Threads::Threads
        Python::Python
        pybind11::pybind11
)

project(ly.activator)

include_directories(
    ../../utils/Headers
    ../driver/Headers
    ../detector/Headers
    ../predictor/Headers
)

AddCXXModule(
    NAME activator
    
    LINKS
        ${OpenCV_LIBS} 
        ${CERES_LIBRARIES}
        Threads::Threads
        Python::Python
        pybind11::pybind11
)
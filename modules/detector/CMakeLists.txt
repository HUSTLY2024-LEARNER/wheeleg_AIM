project(ly.detector)

include_directories(
    ../driver/Headers
    ../../utils/Headers
)

AddCXXModule(
    NAME detector
    
    LINKS
        ${OpenCV_LIBRARIES}
        openvino::runtime 
        Python::Python
        pybind11::pybind11
)

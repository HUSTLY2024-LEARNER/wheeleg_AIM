project(ly.simulator)

include_directories(../driver/Headers)

AddCXXModule(
    NAME simulator
    
    LINKS
        ${OpenCV_LIBS} 
        Threads::Threads
        OpenSSL::SSL
        jsoncpp
        Python::Python
        pybind11::pybind11
)

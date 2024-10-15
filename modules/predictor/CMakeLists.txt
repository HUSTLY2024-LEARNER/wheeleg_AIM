project(ly.predictor)

AddCXXModule(
    NAME predictor
    
    LINKS
        detector
        driver
        tracker
        solver
        Eigen3::Eigen
        ${OPENCV_LIBRARIES}
        Sophus::Sophus
        Threads::Threads
        Python::Python
        pybind11::pybind11
)

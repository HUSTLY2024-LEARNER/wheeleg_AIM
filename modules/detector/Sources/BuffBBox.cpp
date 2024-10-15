#include "BuffBBox.hpp"

namespace DETECTOR
{
    BuffBBox::BuffBBox(const std::string &model_file)
    {
        initModel(model_file);
    }

    BuffBBox::~BuffBBox()
    {
        
    }

    void BuffBBox::initModel(const std::string &model_file)
    {
        core.set_property("CPU", ov::enable_profiling(true));
        model = core.read_model(model_file);
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor().set_element_type(ov::element::f32);
        ppp.output().tensor().set_element_type(ov::element::f32);
        ppp.build();
        compiled_model = core.compile_model(
            model,
            "CPU",
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
        );
        infer_request = compiled_model.create_infer_request();
    }

    BLines BuffBBox::operator()(const cv::Mat &img)
    {
        
    }

} // namespace DETECTOR

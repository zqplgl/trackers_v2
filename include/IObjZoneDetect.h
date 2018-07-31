#ifndef IOBJZONEDETECTFASTER_H
#define IOBJZONEDETECTFASTER_H

#include<base_struct.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace ZoneDetectYoloV3
{
    class IObjZoneDetectYoloV3
    {
        public:
        virtual void Detect(const cv::Mat& img, ObjectPartOut& objectPartOut, const float confidence_threshold)=0;
        virtual ~IObjZoneDetectYoloV3(){}
    };

    //函数名 CreateDetector
    // @brief 创建识别接口，使用时首先调用
    // @author zhanagqipeng
    // @return VehicleZoneDetectNotCarInterface 类型的接口类指针

    IObjZoneDetectYoloV3 *CreateObjZoneYoloV3Detector(const std::string& model_file, const std::string& weights_file,const int gpu_id);


    // 函数名
    // @brief 关闭识别接口，退出时调用
    // @author zhangqipeng
    // @return void
    void DestroyObjZoneYoloV3Detector(IObjZoneDetectYoloV3* detector);
}
#endif

#pragma once

#include <opencv2/opencv.hpp>
#include <mutex>

class Cameras
{
public:
    enum CameraIndex {
        weapon,
        pole
    };

    static Cameras& getInstance() {
        static Cameras cams;
        return cams;
    }

    cv::Mat read(CameraIndex idx) {
        // 调用方法：Cameras::getInstance().read()
        std::lock_guard<std::mutex> lock(mtxs[static_cast<int>(idx)]);
        cv::Mat frame;
        caps[static_cast<int>(idx)].read(frame);
        return frame;
    }

protected:
    Cameras() : caps{cv::VideoCapture(0),cv::VideoCapture(2)} {
        
    }


private:
    cv::VideoCapture caps[2];
    mutable std::mutex mtxs[2];
};


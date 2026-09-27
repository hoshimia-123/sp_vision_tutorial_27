#pragma once

#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera();
    ~Camera();

    cv::Mat read();

private:
    void *handle_ = nullptr;
};
#include "camera.hpp"

#include <iostream>

#include "hikrobot/include/MvCameraControl.h"

cv::Mat transfer(MV_FRAME_OUT & raw)
{
  MV_CC_PIXEL_CONVERT_PARAM cvt_param;
  cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

  cvt_param.nWidth = raw.stFrameInfo.nWidth;
  cvt_param.nHeight = raw.stFrameInfo.nHeight;

  cvt_param.pSrcData = raw.pBufAddr;
  cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

  cvt_param.pDstBuffer = img.data;
  cvt_param.nDstBufferSize = img.total() * img.elemSize();
  cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  auto pixel_type = raw.stFrameInfo.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  cv::cvtColor(img, img, type_map.at(pixel_type));

  return img;
}

Camera::Camera()
{
  int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);

  std::cout << "EnumDevices ret = " << ret << std::endl;

  if (ret != MV_OK) {
    return;
  }

  std::cout << "Device number = " << device_list.nDeviceNum << std::endl;

  if (device_list.nDeviceNum == 0) {
    std::cout << "No camera found!" << std::endl;
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);

  std::cout << "CreateHandle ret = " << ret << std::endl;

  if (ret != MV_OK) {
    handle_ = nullptr;
    return;
  }

  ret = MV_CC_OpenDevice(handle_);

  std::cout << "OpenDevice ret = " << ret << std::endl;

  if (ret != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  std::cout << "Camera opened successfully!" << std::endl;

  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);

  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);

  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);

  MV_CC_SetFloatValue(handle_, "ExposureTime", 1000);

  MV_CC_SetFloatValue(handle_, "Gain", 20);

  MV_CC_SetFrameRate(handle_, 60);

  ret = MV_CC_StartGrabbing(handle_);

  std::cout << "StartGrabbing ret = " << ret << std::endl;

  if (ret != MV_OK) {
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  std::cout << "Start grabbing successfully!" << std::endl;
}
cv::Mat Camera::read()
{
  if (handle_ == nullptr) {
    std::cout << "Invalid camera handle!" << std::endl;
    return cv::Mat();
  }

  MV_FRAME_OUT raw;
  unsigned int nMsec = 100;

  int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);

  if (ret != MV_OK) {
    std::cout << "GetImageBuffer failed, ret = " << ret << std::endl;

    return cv::Mat();
  }

  cv::Mat img = transfer(raw);

  ret = MV_CC_FreeImageBuffer(handle_, &raw);

  if (ret != MV_OK) {
    return cv::Mat();
  }

  return img;
}
Camera::~Camera()
{
  int ret;

  if (handle_ == nullptr) {
    return;
  }

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    return;
  }

  handle_ = nullptr;
}
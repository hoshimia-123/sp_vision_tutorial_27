#include <iostream>

#include "io/camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

int main()
{
  // 初始化相机、yolo类
  Camera camera;
  auto_aim::YOLO yolo("./configs/yolo.yaml");

  while (1) {
    // 调用相机读取图像
    cv::Mat img = camera.read();
    if (img.empty()) {
      return -1;
    }
    // 调用yolo识别装甲板

    auto armors = yolo.detect(img);

    for (auto & armor : armors) {
      tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0));

      std::string label = auto_aim::COLORS[armor.color] + auto_aim::ARMOR_NAMES[armor.name];

      cv::putText(
        img, label, armor.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }
    // 显示图像
    cv::resize(img, img, {}, 1.5, 1.5);
    cv::imshow("img", img);

    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  return 0;
}

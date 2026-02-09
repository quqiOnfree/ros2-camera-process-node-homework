#ifndef NUMBER_DETECTOR_HPP
#define NUMBER_DETECTOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

/**
 * @brief NumberDetector - 模板化数字检测器
 */
class NumberDetector {
public:
  NumberDetector() = default;
  ~NumberDetector() = default;

  /**
   * @brief 将单个数字样本加入到模板库
   * @param number 要保存的数字标签（0-9）
   * @param src 输入图像（任意通道数，函数内部会转换为灰度）
   * @return 成功插入返回 true；若已存在或未找到有效轮廓返回 false
   */
  bool add(std::uint8_t number, cv::InputArray src) {
    cv::Mat img;
    if (src.channels() == 3) {
      // 将 BGR 彩色图转换为灰度图（减少通道并保留形状信息）
      cv::cvtColor(src, img, cv::COLOR_BGR2GRAY);
    } else {
      // 若已是单通道，直接拷贝以免修改原始输入
      img = src.getMat().clone();
    }
    // 统一尺寸，减少尺度对匹配的影响
    cv::resize(img, img, {800, 800});
    // OTSU 二值化，便于后续边缘检测
    cv::threshold(img, img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // 边缘检测，提取轮廓信息
    cv::Canny(img, img, 50, 150);
    // 闭操作：填充边缘内的小孔，连接相邻边缘
    cv::morphologyEx(img, img, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    std::vector<std::vector<cv::Point>> contours;
    // 查找外部轮廓
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 选择面积最大的轮廓作为样本目标
    auto res = std::ranges::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });
    if (res == contours.end()) {
      return false;
    }
    // 计算矩与 Hu 矩，Hu 矩对旋转/缩放不敏感，可作为形状描述子
    auto moment = cv::moments(*res);
    cv::Mat hu_moment;
    cv::HuMoments(moment, hu_moment);
    // 保存到 map 中，键为数字，值为其 Hu 矩
    auto [iter, bres] =
        m_number_map.emplace(std::make_pair(number, std::move(hu_moment)));
    return bres;
  }

  /**
   * @brief 在输入图像中检测可能的四角（二维码/卡片）区域
   * @param src 输入彩色图像
   * @param points 输出的 4 个点（多边形的四个顶点）
   * @return 找到符合要求的四点返回 true，否则 false
   */
  static bool detect(cv::InputArray src, std::vector<cv::Point> &points) {
    cv::Mat blur_img, gray_img, can_img, dil_img;

    // 转灰度
    cv::cvtColor(src, gray_img, cv::COLOR_BGR2GRAY);
    // 中值滤波，用于去除椒盐噪声
    cv::medianBlur(gray_img, blur_img, 5);
    // 自适应阈值：在局部窗口内计算阈值，适合光照不均匀的场景
    cv::adaptiveThreshold(blur_img, blur_img, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11,
                          2);
    // Canny 边缘检测，检测到边缘后进行形态学闭操作以连接边段
    cv::Canny(blur_img, can_img, 25, 75);
    auto kernel =
        cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size{2, 2});
    cv::morphologyEx(can_img, dil_img, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    // 使用类内封装的 findContours（包含 approxPolyDP）得到多边形近似
    findContours(dil_img, contours);
    // 过滤出面积大且顶点为 4 的四边形候选
    auto range =
        contours |
        std::views::filter([](const std::vector<cv::Point> &contour) {
          return contour.size() == 4 && cv::contourArea(contour) > 10000;
        });
    // 从候选中选择面积最大的四边形
    auto res = std::ranges::max_element(
        range.begin(), range.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });
    if (res == range.end()) {
      return false;
    }
    points = *res;
    return true;
  }

  /**
   * @brief 对候选四点区域进行透视矫正并匹配模板解码数字
   * @param src 输入图像（用于透视变换的原始图）
   * @param points 四角点，顺序非保证，内部会 reorder
   * @param num 输出识别到的数字标签
   * @param confidence 输出置信度（0..1）
   * @param dst 可选输出透视变换后的图像
   * @return 成功识别返回 true，否则 false
   */
  bool decode(cv::InputArray src, const std::vector<cv::Point> &points,
              std::uint8_t &num, double &confidence,
              cv::InputOutputArray dst = cv::noArray()) const {
    if (points.size() != 4) {
      return false;
    }
    // 将四点转换为浮点坐标并重新排序为标准顺序（左上、右上、左下、右下）
    cv::Point2f arr[4] = {points[0], points[1], points[2], points[3]};
    reorder<float>(arr);
    const cv::Point2f standard[4] = {
        {0, 0},
        {1000, 0},
        {0, 1000},
        {1000, 1000},
    };
    // 计算透视变换矩阵并应用，得到标准尺寸的候选图像
    auto matrix = cv::getPerspectiveTransform(arr, standard);
    cv::Mat img;
    cv::warpPerspective(src, img, matrix, {1000, 1000});
    if (!dst.empty()) {
      img.copyTo(dst);
    }
    if (src.channels() == 3) {
      cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    // 二值化 + 边缘检测 + 闭操作（同样的流水线以提取主要轮廓）
    cv::threshold(img, img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Canny(img, img, 50, 150);
    cv::morphologyEx(img, img, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto res = std::ranges::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });
    if (res == contours.end()) {
      return false;
    }
    auto moment = cv::moments(*res);
    cv::Mat hu_moment;
    cv::HuMoments(moment, hu_moment);
    // 计算候选 Hu 矩与所有模板之间的形状距离（matchShapes），选择最小者
    std::vector<std::pair<double, std::uint8_t>> dist_vec;
    dist_vec.reserve(m_number_map.size());
    for (auto &[number, number_hu_moment] : std::as_const(m_number_map)) {
      double dist = cv::matchShapes(hu_moment, number_hu_moment,
                                    cv::CONTOURS_MATCH_I1, 0);
      dist_vec.push_back({dist, number});
    }
    auto it = std::ranges::min_element(dist_vec.begin(), dist_vec.end());
    if (it == dist_vec.end()) {
      return false;
    }
    // 将距离映射为置信度，距离越小置信度越高。使用 tanh 做平滑映射
    double loc_confidence = std::tanh(5.0 / it->first + 1e-9);
    confidence = loc_confidence;
    num = it->second;
    return true;
  }

protected:
  template <class Ty> static void reorder(std::span<cv::Point_<Ty>, 4> arr) {
    if (arr.size() != 4) {
      throw std::exception{};
    }
    std::array<cv::Point_<Ty>, 4> tmp{};
    std::array<int, 4> sumPoints, subPoints;
    for (std::size_t i = 0; i < 4; ++i) {
      sumPoints[i] = arr[i].x + arr[i].y;
      subPoints[i] = arr[i].x - arr[i].y;
    }
    tmp[0] = arr[std::distance(
        sumPoints.begin(),
        std::min_element(sumPoints.begin(), sumPoints.end()))];
    tmp[1] = arr[std::distance(
        subPoints.begin(),
        std::max_element(subPoints.begin(), subPoints.end()))];
    tmp[2] = arr[std::distance(
        subPoints.begin(),
        std::min_element(subPoints.begin(), subPoints.end()))];
    tmp[3] = arr[std::distance(
        sumPoints.begin(),
        std::max_element(sumPoints.begin(), sumPoints.end()))];
    for (std::size_t i = 0; i < tmp.size(); ++i) {
      arr[i] = std::move(tmp[i]);
    }
  }

  static void findContours(cv::InputArray src,
                           std::vector<std::vector<cv::Point>> &poly_contours) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(src, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    poly_contours.resize(contours.size());

    for (std::size_t size = contours.size(), i = 0; i < size; ++i) {
      // 计算轮廓周长或曲线长度
      double peri = cv::arcLength(contours[i], true);
      // 计算拟合曲线（更少的顶点）
      cv::approxPolyDP(contours[i], poly_contours[i], 0.02 * peri, true);
    }
  }

  static void
  drawContours(cv::InputOutputArray dst,
               const std::vector<std::vector<cv::Point>> &contours) {
    for (std::size_t size = contours.size(), i = 0; i < size; ++i) {
      double area = cv::contourArea(contours[i]);
      if (area < 1000.0) {
        continue;
      }
      // 画上轮廓
      cv::drawContours(dst, contours, static_cast<int>(i),
                       cv::Scalar{255, 0, 255}, 2);
      // 计算点集或灰度图像中非零像素的正外接矩形（获取外围能括住内部轮廓的矩形）
      auto rect = cv::boundingRect(contours[i]);
      // 画矩形
      cv::rectangle(dst, rect, cv::Scalar{0, 255, 0}, 2);
    }
  }

private:
  std::map<std::uint8_t, cv::Mat> m_number_map;
};

#endif // NUMBER_DETECTOR_HPP

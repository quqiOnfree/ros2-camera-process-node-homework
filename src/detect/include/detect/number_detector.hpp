#ifndef NUMBER_DETECTOR_HPP
#define NUMBER_DETECTOR_HPP

#include <algorithm>
#include <array>
#include <exception>
#include <format>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <print>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

class NumberDetector {
public:
  NumberDetector() = default;
  ~NumberDetector() = default;

  bool add(std::uint8_t number, cv::InputArray src) {
    cv::Mat img;
    if (src.channels() == 3) {
      cv::cvtColor(src, img, cv::COLOR_BGR2GRAY);
    } else {
      img = src.getMat().clone();
    }
    cv::resize(img, img, {400, 400});
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
    auto [iter, bres] =
        m_number_map.emplace(std::make_pair(number, std::move(hu_moment)));
    return bres;
  }

  static bool detect(cv::InputArray src, std::vector<cv::Point> &points) {
    cv::Mat blur_img, gray_img, can_img, dil_img;

    cv::cvtColor(src, gray_img, cv::COLOR_BGR2GRAY);
    // cv::GaussianBlur(gray_img, blur_img, cv::Size{5, 5}, 5, 5);
    cv::medianBlur(gray_img, blur_img, 5);
    // cv::threshold(blur_img, blur_img, 0, 255, cv::THRESH_BINARY |
    // cv::THRESH_OTSU);
    cv::adaptiveThreshold(blur_img, blur_img, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11,
                          2);
    cv::Canny(blur_img, can_img, 25, 75);
    auto kernel =
        cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size{2, 2});
    cv::morphologyEx(can_img, dil_img, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    findContours(dil_img, contours);
    auto range =
        contours |
        std::views::filter([](const std::vector<cv::Point> &contour) {
          return cv::contourArea(contour) > 1000 && contour.size() == 4;
        });
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

  std::int8_t decode(cv::InputArray src, const std::vector<cv::Point> &points,
                     cv::InputOutputArray dst = cv::noArray()) const {
    if (points.size() != 4) {
      return -1;
    }
    cv::Point2f arr[4] = {points[0], points[1], points[2], points[3]};
    reorder<float>(arr);
    const cv::Point2f standard[4] = {
        {0, 0},
        {800, 0},
        {0, 800},
        {800, 800},
    };
    auto matrix = cv::getPerspectiveTransform(arr, standard);
    cv::Mat img;
    cv::warpPerspective(src, img, matrix, {800, 800});
    if (!dst.empty()) {
      img.copyTo(dst);
    }
    if (src.channels() == 3) {
      cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
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
      return -1;
    }
    auto moment = cv::moments(*res);
    cv::Mat hu_moment;
    cv::HuMoments(moment, hu_moment);
    std::vector<std::pair<double, std::uint8_t>> dist_vec;
    dist_vec.reserve(m_number_map.size());
    for (auto &[number, number_hu_moment] : std::as_const(m_number_map)) {
      double dist = cv::matchShapes(hu_moment, number_hu_moment,
                                    cv::CONTOURS_MATCH_I1, 0);
      dist_vec.push_back({dist, number});
    }
    auto it = std::ranges::min_element(dist_vec.begin(), dist_vec.end());
    if (it == dist_vec.end() || it->first > 10.0) {
      return -1;
    }
    return it->second;
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
  drawContours(cv::InputArray src, cv::InputOutputArray dst,
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

#endif

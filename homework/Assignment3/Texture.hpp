//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        cv::Mat patch;
        cv::getRectSubPix(image_data, cv::Size(1, 1), cv::Point2f(u_img, v_img), patch);  
        // 从原图像中提取提取一个感兴趣的矩形区域图像 参数依次是要提取的原图, 矩形的大小, 矩形在原图中的位置, 输出的图像
        // 非整数象素点坐标采用双线性插值提取 对多通道图像，每个通道独立单独完成提取
        auto color = patch.at<cv::Vec3b>(0, 0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H

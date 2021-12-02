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
    Eigen::Vector3f getRawColor(float u, float v)
    {
        auto color = image_data.at<cv::Vec3b>(v, u);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        auto ui = u * (width);
        auto vi = (1 - v) * (height);

        float umin = floor(ui);
        float umax = ceil(ui);
        float vmin = floor(vi);
        float vmax = ceil(vi);

        auto color1 = getRawColor(umin, vmin);
        auto color2 = getRawColor(umax, vmin);
        auto C1 = color1 * (umax - ui) + color2 * (ui - umin); 

        color1 = getRawColor(umin, vmax);
        color2 = getRawColor(umax, vmax);
        auto C2 = color1 * (umax - ui) + color2 * (ui - umin);

        auto color = C1 * (vmax - vi) + C2 * (vi - vmin);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H

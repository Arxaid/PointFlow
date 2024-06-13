// This file is part of the PointFlow project.
//
// Copyright (c) 2024 Arxaid.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

class DepthFrameExtractor{
private:
    int strobeWidth_ = 640;
    int strobeHeight_ = 480;
    int obstacleZones_ = 9;
    float obstacleThreshold_ = 1.0;
    const int singleZoneWidth_ = strobeWidth_/obstacleZones_;
    const int singleZoneHeight_ = 100;
public:
    std::vector<bool> obstacles;
    /**
     * @brief Depth Frame Extractor object constructor by default.
     */
    DepthFrameExtractor(){};
    /**
     * @brief Depth Frame Extractor object constructor.
     * @param width RealSense strobe width.
     * @param heigth RealSense strobe heigth.
     * @param zones Number of separate zones for which obstacle detection task is being solved.
     * @param threshold Threshold distance in meters at which obstacles will be detected.
     */
    DepthFrameExtractor(int width, int heigth, int zones, float threshold){
        strobeWidth_ = width;
        strobeHeight_ = heigth;
        obstacleZones_ = zones;
        obstacleThreshold_ = threshold;
    }
    /**
     * @brief Depth Frame Extractor object destructor by default.
     */
    ~DepthFrameExtractor(){};
    /**
     * @brief Refills obstacles vector based on current point cloud.
     * @param frames Frames from RealSense pipeline.
     */
    void getObstacles(rs2::frameset* frames){
        const rs2::vertex* vertices;
        rs2::pointcloud pointCloud;
        rs2::points points;

        rs2::depth_frame depthFrame = frames->get_depth_frame();
        points = pointCloud.calculate(depthFrame);
        vertices = points.get_vertices();

        obstacles.clear();

        for(int zone = 0; zone < obstacleZones_; zone++){
            float minDistance = 1000.0;
            for(long row = 0; row < singleZoneHeight_; row++){
                for(long col = 0; col < singleZoneWidth_; col++){
                    long curIndex = strobeWidth_ * (row + (strobeHeight_ - singleZoneHeight_)/2) + zone * singleZoneWidth_ + col;
                    float curDistance = sqrt(powf(vertices[curIndex].x, 2) + powf(vertices[curIndex].y, 2) + powf(vertices[curIndex].z, 2));
                    if (curDistance < minDistance and vertices[curIndex].x != 0.0f){ minDistance = curDistance; }
                }
            }
            obstacles.push_back(minDistance > obstacleThreshold_);
        }
    }
};

class ColorFrameExtractor{
private:
    int strobeWidth_ = 640;
    int strobeHeight_ = 480;
    int HueLowThreshold_ = 100;
    int ValueLowThreshold_ = 100;
    int ValueHighThreshold_ = 120;
    int epsApprox_ = 3;
public:
    /**
     * @brief Color Frame Extractor object constructor by default.
     */
    ColorFrameExtractor(){
        cv::namedWindow("Output", cv::WINDOW_FULLSCREEN);
    };
    /**
     * @brief Color Frame Extractor object constructor.
     * @param width RealSense strobe width.
     * @param heigth RealSense strobe heigth.
     * @param hueLow Hue channel threshold.
     * @param valueLow Value channel low threshold.
     * @param valueHigh Value channel high threshold.
     */
    ColorFrameExtractor(int width, int heigth, int hueLow, int valueLow, int valueHigh){
        cv::namedWindow("Output", cv::WINDOW_FULLSCREEN);
        strobeWidth_ = width;
        strobeHeight_ = heigth;
        HueLowThreshold_ = hueLow;
        ValueLowThreshold_ = valueLow;
        ValueHighThreshold_ = valueHigh;
    }
    /**
     * @brief Color Frame Extractor object destructor by default.
     */
    ~ColorFrameExtractor(){};

    void getContours(rs2::frameset* frames){
        cv::Mat colorFrameBGR;
        cv::Mat colorFrameMask;
        cv::Mat colorFrameHSV[3];
        cv::Mat pixels;
        std::vector<std::vector<cv::Point>> contoursDefault;
        std::vector<std::vector<cv::Point>> polygones;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat contours;
        
        rs2::video_frame videoFrame = frames->get_color_frame();
        colorFrameBGR = cv::Mat(cv::Size(strobeWidth_, strobeHeight_), CV_8UC3);
        memcpy((void*)colorFrameBGR.data, (void*)videoFrame.get_data(), videoFrame.get_data_size());
        cv::cvtColor(colorFrameBGR, colorFrameMask, cv::COLOR_BGR2HSV);
        cv::split(colorFrameMask, colorFrameHSV);

        pixels = cv::Mat(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        for(int row = 0; row < videoFrame.get_height(); row++){
            for(int col = 0; col < videoFrame.get_width(); col++){
                pixels.at<uint8_t>(row, col) = 0;
                if ((colorFrameHSV[2].at<uint8_t>(row, col) < ValueLowThreshold_ 
                    and colorFrameHSV[0].at<uint8_t>(row, col) >=  HueLowThreshold_) 
                    and colorFrameHSV[2].at<uint8_t>(row, col) <= ValueHighThreshold_)
                    {
                        pixels.at<uint8_t>(row, col) = 255;
                    }
            }
        }
        
        cv::imshow("Output", pixels);
        cv::waitKey(1);

        // contours = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        // cv::findContours(contours, contoursDefault, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        // cv::approxPolyDP(contoursDefault, polygones, epsApprox_, true);
        // for(int counter = 0; counter < polygones.size(); counter++){
        //     if (polygones[counter].size() < 5){
        //         cv::drawContours(contours, polygones, counter, cv::Scalar(255, 0, 0), 3, cv::LINE_8, hierarchy, 0);
        //     }
        // }
    }
};
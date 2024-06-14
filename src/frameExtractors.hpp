// This file is part of the PointFlow project.
//
// Copyright (c) 2024 Arxaid.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "handlers.hpp"

class DepthFrameExtractor{
private:
    std::vector<bool> obstacles_;

    int strobeWidth_ = 640;
    int strobeHeight_ = 480;
    int obstacleZones_ = 9;
    float obstacleThreshold_ = 1.0;
    const int singleZoneWidth_ = strobeWidth_/obstacleZones_;
    const int singleZoneHeight_ = 100;
public:
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
     * @return std::vector<bool> Boolean vector of separate zones size, the elements of which are interpreted as presence/absence of an obstacle in zones.
     */
    std::vector<bool> getObstacles(rs2::frameset* frames){
        const rs2::vertex* vertices;
        rs2::pointcloud pointCloud;
        rs2::points points;

        rs2::depth_frame depthFrame = frames->get_depth_frame();
        points = pointCloud.calculate(depthFrame);
        vertices = points.get_vertices();

        obstacles_.clear();
        for(int zone = 0; zone < obstacleZones_; zone++){
            float minDistance = 1000.0;
            for(long row = 0; row < singleZoneHeight_; row++){
                for(long col = 0; col < singleZoneWidth_; col++){
                    long curIndex = strobeWidth_ * (row + (strobeHeight_ - singleZoneHeight_)/2) + zone * singleZoneWidth_ + col;
                    float curDistance = sqrt(powf(vertices[curIndex].x, 2) + powf(vertices[curIndex].y, 2) + powf(vertices[curIndex].z, 2));
                    if (curDistance < minDistance and vertices[curIndex].x != 0.0f){ minDistance = curDistance; }
                }
            }
            obstacles_.push_back(minDistance > obstacleThreshold_);
        }
        return obstacles_;
    }
};

class ColorFrameExtractor{
private:
    cv::Mat hueChannelRaw_;
    cv::Mat saturationChannelRaw_;
    cv::Mat valueChannelRaw_;

    cv::Mat hueChannelMasked_;
    cv::Mat saturationChannelMasked_;
    cv::Mat valueChannelMasked_;

    cv::Mat colorFrameBGR_;
    cv::Mat colorFrameHSV_;
    cv::Mat colorFrameMasked_;

    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;

    std::vector<std::vector<cv::Point>> obstacleZones_;

    int strobeWidth_ = 640;
    int strobeHeight_ = 480;
    int zoneHeigth_ = 100;

    int polyEpsilon_ = 10;
    int valuableContoursNumber_ = 1;

    int hueLowThreshold_ = 90;
    int hueHighThreshold_ = 152;
    int saturationLowThreshold_ = 150;
    int saturationHighThreshold_ = 255;
    int valueLowThreshold_ = 104;
    int valueHighThreshold_ = 255;
public:
    /**
     * @brief Color Frame Extractor object constructor by default.
     */
    ColorFrameExtractor(){};
    /**
     * @brief Color Frame Extractor object constructor.
     * @param width RealSense strobe width.
     * @param heigth RealSense strobe heigth.
     */
    ColorFrameExtractor(int width, int heigth){
        strobeWidth_ = width;
        strobeHeight_ = heigth;
    }
    /**
     * @brief Color Frame Extractor object constructor.
     * @param width RealSense strobe width.
     * @param heigth RealSense strobe heigth.
     * @param hueLow Hue channel low threshold.
     * @param hueHigh Hue channel high threshold.
     * @param valueLow Saturation channel low threshold.
     * @param valueLow Saturation channel high threshold.
     * @param valueLow Value channel low threshold.
     * @param valueHigh Value channel high threshold.
     * @param contoursNumber Valuable contours number.
     */
    ColorFrameExtractor(int width, int heigth, int hueLow, int hueHigh, int saturationLow, 
                        int saturationHigh, int valueLow, int valueHigh, int contoursNumber){
        strobeWidth_ = width;
        strobeHeight_ = heigth;
        hueLowThreshold_ = hueLow;
        hueHighThreshold_ = hueHigh;
        saturationLowThreshold_ = saturationLow;
        saturationHighThreshold_ = saturationHigh;
        valueLowThreshold_ = valueLow;
        valueHighThreshold_ = valueHigh;
        valuableContoursNumber_ = contoursNumber;          
    }
    /**
     * @brief Color Frame Extractor object destructor by default.
     */
    ~ColorFrameExtractor(){};
    /**
     * @brief Refills HSV channels based on current BGR color frame.
     * @param frames Frames from RealSense pipeline.
     */
    void getColorFrame(rs2::frameset* frames){
        std::vector<cv::Mat> channelsHSV(3);

        rs2::video_frame videoFrame = frames->get_color_frame();
        colorFrameBGR_ = cv::Mat(cv::Size(strobeWidth_, strobeHeight_), CV_8UC3);
        memcpy((void*)colorFrameBGR_.data, (void*)videoFrame.get_data(), videoFrame.get_data_size());
        cv::cvtColor(colorFrameBGR_, colorFrameHSV_, cv::COLOR_BGR2HSV);

        cv::split(colorFrameHSV_, channelsHSV);
        hueChannelRaw_ = channelsHSV[0];
        saturationChannelRaw_ = channelsHSV[1];
        valueChannelRaw_ = channelsHSV[2];
    }
    /**
     * @brief Creates selection mask in accordance with specified thresholds and applies it to the HSV color frame.
     */
    void applySelectionMask(){
        cv::Mat hueMaskHigh = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        cv::Mat hueMaskLow = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        cv::threshold(hueChannelRaw_, hueMaskHigh, hueHighThreshold_, 255, cv::THRESH_BINARY_INV);
        cv::threshold(hueChannelRaw_, hueMaskLow, hueLowThreshold_, 255, cv::THRESH_BINARY);
        hueMaskLow.copyTo(hueChannelMasked_, hueMaskHigh);

        cv::Mat saturationMaskHigh = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        cv::Mat saturationMaskLow = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        cv::threshold(saturationChannelRaw_, saturationMaskHigh, saturationHighThreshold_, 255, cv::THRESH_BINARY_INV);
        cv::threshold(saturationChannelRaw_, saturationMaskLow, saturationLowThreshold_, 255, cv::THRESH_BINARY);
        saturationMaskLow.copyTo(saturationChannelMasked_, saturationMaskHigh);

        cv::Mat valueMaskHigh = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        cv::Mat valueMaskLow = cv::Mat::zeros(cv::Size(strobeWidth_, strobeHeight_), CV_8UC1);
        cv::threshold(valueChannelRaw_, valueMaskHigh, valueHighThreshold_, 255, cv::THRESH_BINARY_INV);
        cv::threshold(valueChannelRaw_, valueMaskLow, valueLowThreshold_, 255, cv::THRESH_BINARY);
        valueMaskLow.copyTo(valueChannelMasked_, valueMaskHigh);

        cv::Mat temporaryMask;
        cv::Mat fullMask;
        hueChannelMasked_.copyTo(temporaryMask, saturationChannelMasked_);
        temporaryMask.copyTo(fullMask, valueChannelMasked_);
        colorFrameMasked_ = fullMask;
    }
    /**
     * @brief Finds contours in the masked HSV color frame and approximates them polynomially.
     */
    void getContours(){
        cv::findContours(colorFrameMasked_, contours_, hierarchy_, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        std::sort(contours_.begin(), contours_.end(), compHandler);
        std::vector<std::vector<cv::Point>> temporaryContours;
        if (contours_.size() >= valuableContoursNumber_){
            for (int counter = 0; counter < valuableContoursNumber_; counter++){ 
                temporaryContours.push_back(contours_[counter]); 
            }
            contours_ = temporaryContours;
        }
        for(int counter = 0; counter < contours_.size(); counter++){
            cv::approxPolyDP(cv::Mat(contours_[counter]), contours_[counter], polyEpsilon_, true);
        }
    } 
    /**
     * @return cv::Point Center of squared color mark contour.
     */
    cv::Point getMarkCenter(){
        for(std::vector<cv::Point> contour: contours_){
            cv::Point singleMarkCenter = cv::Point(0,0);
            for(cv::Point point: contour) singleMarkCenter += point;
            singleMarkCenter /= 4;
            return singleMarkCenter;
        }
    }
    /**
     * @brief Navigation and obstacles detection algorithms visualization.
     */
    void visualize(std::vector<bool> obstacles){
        cv::Mat resultFrame;
        colorFrameBGR_.copyTo(resultFrame, colorFrameMasked_);
        for(int counter = 0; counter < contours_.size(); counter++){
            cv::drawContours(resultFrame, contours_, counter, cv::Scalar(255, 0, 0), 2, cv::LINE_8, hierarchy_, 0);
        }
        cv::line(resultFrame, cv::Point(strobeWidth_/2, 0), cv::Point(strobeWidth_/2, strobeHeight_), cv::Scalar(255, 255, 255), 1, cv::LINE_8);
        std::vector<std::vector<cv::Point>> zones;
        for(int zone = 0; zone < obstacles.size(); zone++){
            std::vector<cv::Point> zoneContour;
            zoneContour.push_back(cv::Point(zone * strobeWidth_/obstacles.size(), (strobeHeight_ - zoneHeigth_)/2));
            zoneContour.push_back(cv::Point(zone * strobeWidth_/obstacles.size() + strobeWidth_/obstacles.size() - 1, (strobeHeight_ - zoneHeigth_)/2));
            zoneContour.push_back(cv::Point(zone * strobeWidth_/obstacles.size() + strobeWidth_/obstacles.size() - 1, (strobeHeight_ + zoneHeigth_)/2));
            zoneContour.push_back(cv::Point(zone * strobeWidth_/obstacles.size(), (strobeHeight_ + zoneHeigth_)/2));
            zones.push_back(zoneContour);
        }
        for(int counter = 0; counter < obstacles.size(); counter++){
            if (obstacles[counter]){ cv::drawContours(resultFrame, zones, counter, cv::Scalar(0, 255, 0), 1, cv::LINE_8); }
            else { cv::drawContours(resultFrame, zones, counter, cv::Scalar(0, 0, 255), 1, cv::LINE_8); }
        }

        cv::imshow("Output", resultFrame);
        cv::waitKey(1);
    }
};
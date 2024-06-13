// This file is part of the PointFlow project.
//
// Copyright (c) 2024 Arxaid.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <librealsense2/rs_advanced_mode.hpp>
#include <fstream>
#include <iostream>
#include <string.h>

class PipelineMaker{
private:
    int strobeWidth_ = 640;
    int strobeHeight_ = 480;
    int framerate_ = 30;
    bool colorChannel_ = false;
    bool depthChannel_ = false;
public:
    /**
     * @brief Pipeline Maker object constructor by default.
     */
    PipelineMaker(){};
    /**
     * @brief Pipeline Maker object constructor.
     * @param width RealSense strobe width.
     * @param heigth RealSense strobe heigth.
     * @param framerate Stream frames per second value.
     * @param color Enable/Disable color pipeline.
     * @param depth Enable/Disable depth pipeline.
     */
    PipelineMaker(int width, int heigth, int framerate, bool color, bool depth){
        strobeWidth_ = width;
        strobeHeight_ = heigth;
        framerate_ = framerate;
        colorChannel_ = color;
        depthChannel_ = depth;
    }
    /**
     * @brief Pipeline Maker object object destructor by default.
     */
    ~PipelineMaker(){};
    /**
     * @brief Configures Pipeline from *.json.
     * @param configPath Path to RealSense *.json configuration file.
     */
    void configurePipeline(std::string configPath){
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) throw "No RealSense devices found.";
        if (devices[0].is<rs400::advanced_mode>()){
            auto advancedModeDev = devices[0].as<rs400::advanced_mode>();
            std::ifstream t(configPath);
            std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
            advancedModeDev.load_json(str);
        }
    }
    /**
     * @brief Makes Pipeline according to the config.
     * @param pipe rs2 pipeline object.
     */
    void makePipeline(rs2::pipeline& pipe){
        rs2::config* config = new rs2::config();
        if (colorChannel_) config->enable_stream(RS2_STREAM_COLOR, strobeWidth_, strobeHeight_, RS2_FORMAT_BGR8, framerate_); 
        if (depthChannel_) config->enable_stream(RS2_STREAM_DEPTH, strobeWidth_, strobeHeight_, RS2_FORMAT_Z16,  framerate_);
        pipe.start(*config);
    }
};
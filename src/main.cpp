// This file is part of the PointFlow project.
//
// Copyright (c) 2024 Arxaid.
// Licensed under the Apache License, Version 2.0.

#include "pipeline.hpp"
#include "frameExtractors.hpp"

int main(int argc, char* argv[]){

    int exitKey = cv::waitKey(10);

    int strobeWidth = 640;
    int strobeHeight = 480;
    int framerate = 30;
    std::string configPath = "../config/configRS-Stereo-RGB.json";

    int obstaclesZones = 9;
    float obstaclesThreshold = 1.0;

    int hue = 100;
    int valueLow = 100;
    int valueHigh = 120;

    PipelineMaker pipeMaker = PipelineMaker(
        strobeWidth, strobeHeight, framerate, true, true);
    try{ pipeMaker.configurePipeline(configPath); }
    catch(const char* err){ std::cerr << err << "\n"; }
    rs2::pipeline pipe;
    pipeMaker.makePipeline(pipe);

    DepthFrameExtractor depthExtractor = DepthFrameExtractor(
        strobeWidth, strobeHeight, obstaclesZones, obstaclesThreshold);

    ColorFrameExtractor colorExtractor = ColorFrameExtractor(
        strobeWidth, strobeHeight, hue, valueLow, valueHigh);


    rs2::frameset frames;
    while(true){
        frames = pipe.wait_for_frames();
        depthExtractor.getObstacles(&frames);
        for (int i=0; i<depthExtractor.obstacles.size(); i++){
            std::cout << depthExtractor.obstacles[i] << " ";
        }
        std::cout << "\n";

        colorExtractor.getContours(&frames);
    }
    return 0;
}
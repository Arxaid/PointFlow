// This file is part of the PointFlow project.
//
// Copyright (c) 2024 Arxaid.
// Licensed under the Apache License, Version 2.0.

#include "pipeline.hpp"
#include "frameExtractors.hpp"

int main(int argc, char* argv[]){
    
    int strobeWidth = 640;
    int strobeHeight = 480;
    int framerate = 30;
    std::string configPath = "../config/configRS-Stereo-RGB.json";

    int obstaclesZones = 9;
    float obstaclesThreshold = 1.0;

    PipelineMaker pipeMaker = PipelineMaker(
        strobeWidth, strobeHeight, framerate, true, true);

    try{ pipeMaker.configurePipeline(configPath); }
    catch(const char* err){ std::cerr << err << "\n"; }
    rs2::pipeline pipe;
    pipeMaker.makePipeline(pipe);

    DepthFrameExtractor depthExtractor = DepthFrameExtractor(
        strobeWidth, strobeHeight, obstaclesZones, obstaclesThreshold);

    ColorFrameExtractor colorExtractor = ColorFrameExtractor(
        strobeWidth, strobeHeight);

    rs2::frameset frames;
    while(true){
        frames = pipe.wait_for_frames();
        auto obstacles = depthExtractor.getObstacles(&frames);

        std::cout << "Obstacles: ";
        for(int counter = 0; counter < obstacles.size(); counter++){
            std::cout << obstacles[counter] << " ";
        }

        colorExtractor.getColorFrame(&frames);
        colorExtractor.applySelectionMask();
        colorExtractor.getContours();
        colorExtractor.visualize(obstacles);

        int misalignment = strobeWidth/2 - colorExtractor.getMarkCenter().x;
        std::cout << "\nColor mark misalignment: " << misalignment << "\n";
        std::cout << "=================================================\n";
    }
    return 0;
}
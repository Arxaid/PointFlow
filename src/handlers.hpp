// This file is part of the PointFlow project.
//
// Copyright (c) 2024 Arxaid.
// Licensed under the Apache License, Version 2.0.

#include <opencv2/opencv.hpp>

bool compHandler(const std::vector<cv::Point>& a, const std::vector<cv::Point>& b){ return a.size() > b.size(); }
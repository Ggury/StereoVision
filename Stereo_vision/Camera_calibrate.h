#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/imgcodecs.hpp"
#include <vector>

void Calibrate_Camera(int width, int height);

void _DeptMap();

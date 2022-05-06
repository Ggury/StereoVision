#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include "Camera_calibrate.h"

using namespace std;
using namespace cv;


int main() {
    char action;

    cout << "Hello, World!" << endl;
    cout << "OpenCV version is " << CV_VERSION << endl;
    cin >> action;
    if (action == 'c')
    {
        Calibrate_Camera(3, 7);
    }
    if (action == 'd')
    {
        _DeptMap();
        /*
        FileStorage matx;
        matx.open("STEREOMATRIX.xml", FileStorage::READ);
        Mat left_s_x, left_s_y;
        Mat right_s_x, right_s_y;
        matx["Left_SMat_x"] >> left_s_x;
        matx["Left_SMat_y"] >> left_s_y;
        matx["Right_SMat_x"] >> right_s_x;
        matx["Right_SMat_y"] >> right_s_y;
        Mat cam_matrix_1;
        Mat cam_matrix_2;
        Mat gray_L;
        Mat gray_R;
        Ptr<StereoBM> bm;
        Mat disp, disp8, colored;

        while (1)
        {
            Mat left_calibrated, right_calibrated;
            VideoCapture cap_L(1);
            VideoCapture cap_R(2);
            cap_L >> cam_matrix_1;
            cvtColor(cam_matrix_1, gray_L, COLOR_BGR2GRAY);
            cap_R >> cam_matrix_2;
            cvtColor(cam_matrix_2, gray_R, COLOR_BGR2GRAY);
            cv::remap(gray_L,left_calibrated,left_s_x,left_s_y,cv::INTER_LANCZOS4,BORDER_CONSTANT,0);
            cv::remap(gray_R, right_calibrated, right_s_x, right_s_y, cv::INTER_LANCZOS4, BORDER_CONSTANT, 0);
            bm -> compute(left_calibrated, right_calibrated, disp);
            disp.convertTo(disp8, CV_8U);
            applyColorMap(disp8, colored, cv::COLORMAP_JET);
        }*/

    }
    /*for (FileNodeIterator cur = n.begin(); cur != n.end(); cur++)
    {
        FileNode item = *cur;
        Mat v;
        //item["Left_SMat_x"]->v;
    }*/

    return 0;
}
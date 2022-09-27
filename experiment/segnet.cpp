#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <thread>
#include <time.h>
#include "common.h"
#include "segnet.h"

using namespace std;

int main(int argc, char** argv)
{
    // Load network
    Classifier classifier;
    string colorfile = "../models/color.png";
    cv::Mat color = cv::imread(colorfile, 1);

    // Load image
    for (int i = 1550; i < 2309; i++)
    {
        char file_name[256];

#if (datasetvalue==0)
        //sprintf(file_name, "../dataset/05/image_3/%06d.png",
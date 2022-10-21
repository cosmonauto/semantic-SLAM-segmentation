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
        //sprintf(file_name, "../dataset/05/image_3/%06d.png",i);
        sprintf(file_name, "/home/relaybot/Mu_Link/KittiData/05/image_2/%06d.png",i);
#elif (datasetvalue==1)
        sprintf(file_name, "../Image_Test/mu/rgb/%04d.jpg",i);
#else
        cout<<"没有定义数据集"<<endl;
#endif

        cv::Mat frame = cv::imread(file_name, 1);
        //cv::resize(frame, frame, cv::Size(960,720));

//【1】CV_8UC1---则可以创建----8位无符号的单通道---灰度图片------grayImg
//#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
//#define CV_8UC2 CV_MAKETYPE(CV_8U,2)
//【2】CV_8UC3---则可以创建----8位无符号的三通道---RGB彩色图像---colorImg
//#define CV_8UC3 CV_MAKETYPE(CV_8U,3)
//【3】CV_8UC4--则可以创建-----8位无符号的四通道---带透明色的RGB图像
//#define CV_8UC4 CV_MAKETYPE(CV_8U,4)

        if(frame.size().width<=0)continue;

        cv::Mat copy_frame = frame.clone();

        // time
        clock_t starttime=clock();

        // Prediction
        cv::imshow("frame", frame);
        cv::Mat segnet_frame ;
        cv::resize(frame, segnet_frame, cv::Size(480,360));

        std::vector<Prediction> predictions = classifier.Classify(segnet_frame);

        //------------------------------------------------------------------------
        //std::vector<Prediction> predictions = classifier.Classify(new_frame);
        /* 在Caffe-Segnet的函数中，主要是Predict此部分代码
         * std::vector<float> output = Predict(img);
         * 输出的容器vector大小为宽*高，代表每个像素点的分类结果输出
         * 而predictions.push_back(std::make_pair(labels_[idx], idx));
         * 上述代码其实并没有将Label与输出的结果idx关联起来，输出还是按照0-11排序
         * 所以下面的代码通过Label 的if判断去改变second(idx)实际上没有改变其因素
         */

        string Predictions_name[360][480];
        string Predictions_num[360][480];
        for(int i_1=0;i_1<360;i_1++)
        {
            for(int j_1=0;j_1<480;j_1++)
            {
                Predictions_name[i_1][j_1]=predictions[i_1*480+j_1].first;
                Predictions_num[i_1][j_1]=predictions[i_1*480+j_1].second;
                if(predictions[i_1*480+j_1].first=="Pavement")
                {
                    predictions[i_1*480+j_1].second = 4;
       
#ifndef LOOPER_H
#define LOOPER_H

/**
  * Looper.h
  * The loop closure detector based on dbow2;
  */

#include "common_headers.h"
#include "rgbdframe.h"
#include "converter.h"

#include <opencv2/opencv.hpp>

#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Looper
{
public:

    Looper( const ParameterReader& para )
        : parameterReader( para )
    {
        string  vocab_file = para.getData<string>("looper_vocab_file");
        cout<<"loading vocabulary file, this may take a while..."<<endl;
        vocab.loadFromTextFile( vocab_file );
        cout<<"load ok."<<endl;

        min_sim_score = para.getData<float>("looper_min_sim_score");
        min_interval = para.getData<int>("looper_min_interval");
    }

    // 往数据库里增加一条frame记录
    void add( RGBDFrame::Ptr& frame )
    {
        vector<cv::Mat> desps = frame->getAllDescriptorsVec();
        DBoW2::FeatureVector featVec;
        vocab.transform( desps, frame->b
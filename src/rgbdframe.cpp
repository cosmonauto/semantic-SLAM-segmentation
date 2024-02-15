
#include "rgbdframe.h"
#include "common_headers.h"
#include "parameter_reader.h"
#include "stereo.h"

using namespace rgbd_tutor;

RGBDFrame::Ptr   FrameReader::next()
{
    switch (dataset_type) {
    case NYUD:
        //TODO 增加nyud的接口
        break;
    case TUM:
    {
        if (currentIndex < start_index || currentIndex >= rgbFiles.size())
            return nullptr;

        RGBDFrame::Ptr   frame (new RGBDFrame);
        frame->id = currentIndex;
        frame->rgb = cv::imread( dataset_dir + rgbFiles[currentIndex]);
        frame->depth = cv::imread( dataset_dir + depthFiles[currentIndex], CV_LOAD_IMAGE_UNCHANGED);

        if (frame->rgb.data == nullptr || frame->depth.data==nullptr)
        {
            // 数据不存在
            return nullptr;
        }

        frame->camera = this->camera;
        currentIndex ++;
        return frame;
    }    
    case KITTI:
    {
        if (currentIndex < start_index || currentIndex >= rgbFiles.size() || currentIndex >= parameterReader.getData<int>("end_index"))
            return nullptr;

        string rgb_dir = parameterReader.getData<string>("rgb_dir"); 
        string depth_dir = parameterReader.getData<string>("depth_dir"); 

	// RGB
        RGBDFrame::Ptr   frame (new RGBDFrame);
        frame->id = currentIndex;
        frame->rgb = cv::imread(dataset_dir + rgb_dir + rgbFiles[currentIndex+1]);

//	frame->rgb_cur_r = cv::imread(dataset_dir + "image_1/" + rgbFiles[currentIndex+1], 1);
//	frame->rgb_pre_r = cv::imread(dataset_dir + "image_1/" + rgbFiles[currentIndex], 1);
//
//	// libviso2
//	frame->img_lc = cv::imread(dataset_dir + "image_0/" + rgbFiles[currentIndex+1], 0);
//	frame->img_lp = cv::imread(dataset_dir + "image_0/" + rgbFiles[currentIndex], 0);
//	frame->img_rc = cv::imread(dataset_dir + "image_1/" + rgbFiles[currentIndex+1], 0);
//	frame->img_rp = cv::imread(dataset_dir + "image_1/" + rgbFiles[currentIndex], 0);
//
//	// motion
//	frame->moving_mask = cv::Mat::zeros(frame->img_lc.rows, frame->img_lc.cols, CV_8UC1);
//
//	// Depth
//	cv::Mat img_l = cv::imread(dataset_dir + "image_0/" + rgbFiles[currentIndex+1], 0);
//	cv::Mat img_r = cv::imread(dataset_dir + "image_1/" + rgbFiles[currentIndex+1], 0);


        frame->rgb_cur_r = cv::imread(dataset_dir + "image_3/" + rgbFiles[currentIndex+1], 1);
        frame->rgb_pre_r = cv::imread(dataset_dir + "image_3/" + rgbFiles[currentIndex], 1);

        // libviso2
        frame->img_lc = cv::imread(dataset_dir + "image_2/" + rgbFiles[currentIndex+1], 0);
        frame->img_lp = cv::imread(dataset_dir + "image_2/" + rgbFiles[currentIndex], 0);
        frame->img_rc = cv::imread(dataset_dir + "image_3/" + rgbFiles[currentIndex+1], 0);
        frame->img_rp = cv::imread(dataset_dir + "image_3/" + rgbFiles[currentIndex], 0);

        // motion
        frame->moving_mask = cv::Mat::zeros(frame->img_lc.rows, frame->img_lc.cols, CV_8UC1);

        // Depth
        cv::Mat img_l = cv::imread(dataset_dir + "image_2/" + rgbFiles[currentIndex+1], 0);
        cv::Mat img_r = cv::imread(dataset_dir + "image_3/" + rgbFiles[currentIndex+1], 0);


	cv::Mat disp_sgbm; 
	calDisparity_SGBM(img_l, img_r, disp_sgbm);
	frame->disparity = disp_sgbm.clone();

	double minDisparity = FLT_MAX; cv::minMaxIdx(disp_sgbm, &minDisparity, 0, 0, 0 );
	frame->depth = cv::Mat(img_l.size(), CV_16UC1, cv::Scalar(0));
	double baseline = parameterReader.getData<double>("camera.baseline");
	double cu = parameterReader.getData<double>("camera.cx");
	double cv = parameterReader.getData<double>("camera.cy");
	double f = parameterReader.getData<double>("camera.fx");
	double roix = parameterReader.getData<double>("camera.roix");
	double roiy = parameterReader.getData<double>("camera.roiy");
	double roiz = parameterReader.getData<double>("camera.roiz");
	double scale = parameterReader.getData<double>("camera.scale");

	for (int v = 0; v < img_l.rows; ++v)
	{
		ushort* depth_img_ptr = frame->depth.ptr<ushort>(v);
		const short* disparity_ptr = disp_sgbm.ptr<short>(v);
		for (int u = 0; u < img_l.cols; ++u)
		{
			short d = disparity_ptr[u];
			if (fabs(d)>FLT_EPSILON) //remove moving objects and outside the ROI
			{
				double pw = baseline/(1.0*static_cast<double>(d));
				double px = ((static_cast<double>(u)-cu)*pw)*16.0f;
				double py = ((static_cast<double>(v)-cv)*pw)*16.0f;
				double pz = (f*pw)*16.0f;

		    		if (fabs(d-minDisparity) <= FLT_EPSILON)
		    			continue;
    				if (fabs(px)<roix && fabs(py)<roiy && fabs(pz)<roiz && pz>0)//outside the ROI
					depth_img_ptr[u] = ushort(pz * scale);
			}
		}
	}

	// Semantic now
//	cv::Mat new_frame;
//	cv::resize(frame->rgb, new_frame, cv::Size(480,360));
//	std::vector<Prediction> predictions = classifier.Classify(new_frame);
//	cv::Mat segnet(new_frame.size(), CV_8UC3, cv::Scalar(0,0,0));
//	for (int i = 0; i < 360; ++i)
//	{	
//		uchar* segnet_ptr = segnet.ptr<uchar>(i);
//		for (int j = 0; j < 480; ++j)
//		{
//			segnet_ptr[j*3+0] = predictions[i*480+j].second;
//			segnet_ptr[j*3+1] = predictions[i*480+j].second;
//			segnet_ptr[j*3+2] = predictions[i*480+j].second;
//		}
//	}
//	resize(segnet, segnet, frame->rgb.size());
//	cv::cvtColor(segnet, frame->raw_semantic, CV_BGR2GRAY);;
//	cv::LUT(segnet, frame->color, segnet);
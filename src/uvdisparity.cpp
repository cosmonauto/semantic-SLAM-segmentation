
/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "uvdisparity.hpp"
#include "stereo.h"
using namespace cv;
using namespace std;








USegmentPars::USegmentPars(int min_intense_, int min_disparity_raw_, int min_area_)
{
    min_intense = min_intense_;
    min_disparity_raw = min_disparity_raw_;
    min_area = min_area_;
}

//constructer
 UVDisparity::UVDisparity()
 {
     out_th_ = 6.0f;                    //outlier rejection threshold
     inlier_tolerance_ = 3;             //inlier tolerance threshold
     min_adjust_intense_ = 19;          //minum adjust intensity


     //init Kalman filter parameters
     pitch1_KF = new KalmanFilter(2,1,0);
     pitch1_KF->transitionMatrix = *(Mat_<float>(2, 2) << 1,0,0,1);
     cv::setIdentity(pitch1_KF->measurementMatrix);
     cv::setIdentity(pitch1_KF->processNoiseCov, Scalar::all(0.000005));
     cv::setIdentity(pitch1_KF->measurementNoiseCov, Scalar::all(0.001));
     cv::setIdentity(pitch1_KF->errorCovPost, Scalar::all(1));

     pitch2_KF = new KalmanFilter(2,1,0);
     pitch2_KF->transitionMatrix = *(Mat_<float>(2, 2) << 1,0,0,1);
     cv::setIdentity(pitch2_KF->measurementMatrix);
     cv::setIdentity(pitch2_KF->processNoiseCov, Scalar::all(0.000005));
     cv::setIdentity(pitch2_KF->measurementNoiseCov, Scalar::all(0.001));
     cv::setIdentity(pitch2_KF->errorCovPost, Scalar::all(1));

     pitch1_measure.create(1,1,CV_32F);
     pitch2_measure.create(1,1,CV_32F);


 }

 //deconstructor
 UVDisparity::~UVDisparity()
 {
     delete pitch1_KF,pitch2_KF;
 }


 /*Since the reconstructed 3D points sometimes not correspond the
  *triangulated coordinates (the reason is disparity map is smoothed
  *while the disparity between two matched points is not smoothed),
  *we re-calculate the 3D position, and filter out the inliers outsid
  *e the ROI
*/
void UVDisparity::filterInOut(const Mat &image, const Mat &roi_mask,const Mat &sgbm_roi,
                              VisualOdometryStereo& vo, const double pitch)
{
  /*calibration parameters*/
  double f = calib_.f;
  double cu = calib_.c_x;
  double cv = calib_.c_y;
  double base = calib_.b;

  double cos_p = cos(pitch);
  double sin_p = sin(pitch);
  int threshold = -3000;

  cv::Mat motion1 = vo.getMotion();
  cv::Mat show_in,show_out,xyz0,xyz1;
  
  cvtColor(image, show_in, CV_GRAY2BGR);
  cvtColor(image, show_out, CV_GRAY2BGR);

  vector<pmatch>::iterator it_in = vo.quadmatches_inlier.begin();
  //cout<<"the inliers: "<<vo.quadmatches_inlier.size()<<endl;


  for(; it_in!=vo.quadmatches_inlier.end(); )
  {
    /*current feature point position*/
    int uc = (*it_in).u1c;
    int vc = (*it_in).v1c;
  
    /*previous feature point position*/
    int up = (*it_in).u1p;
    int vp = (*it_in).v1p;
  
    if(roi_mask.at<uchar>(vc,uc) > 0)
    {
      double d = max(uc - (*it_in).u2c, 1.0f);
      double yc = (vc - cv)*base/d;
      double zc = f*base/d;

      yc = cos_p * yc + sin_p * zc;
  
       if(true)
      {
        cv::circle(show_in, cv::Point(uc,vc),2,cv::Scalar(0,0,255),2,8,0);
        cv::circle(show_in, cv::Point(up,vp),2,cv::Scalar(0,0,255),2,8,0);
        cv::line(show_in,  cv::Point(up,vp),  cv::Point(uc,vc), cv::Scalar(0,255,0),1,8,0);

        (*it_in).dis_c = sgbm_roi.at<short>(vc,uc);

        it_in++;
      }
       else
       {
         it_in = vo.quadmatches_inlier.erase(it_in);
       }
       
    }
    else
    {
      it_in = vo.quadmatches_inlier.erase(it_in);
    }
  }
  //cout<<"After the filter process, the number of inlier is: "<<vo.p_matched_inlier.size()<<endl;
  vector<pmatch>::iterator it_out = vo.quadmatches_outlier.begin();
 // cout<<"before the filter process, the number of outlier is: "<<vo.p_matched_outlier.size()<<endl;
  for(; it_out!=vo.quadmatches_outlier.end(); )
  {
    
    int uc = (*it_out).u1c;
    int vc = (*it_out).v1c;
    //int vc2 = (*it).v2c;

    int up = (*it_out).u1p;
    int vp = (*it_out).v1p;
      
    if(roi_mask.at<uchar>(vc,uc) > 0)
    {
      double d = max(uc - (*it_out).u2c, 1.0f);            
      double xc = (uc - cu)*base/d;
      double yc = (vc - cv)*base/d;
      double zc = f*base/d;
      
      yc = cos_p * yc + sin_p * zc;
      zc = cos_p * zc - sin_p * yc;
            
      if(xc > threshold)
      {
        cv::circle(show_in, cv::Point(uc,vc),2,cv::Scalar(255,0,0),2,8,0);
        cv::circle(show_in, cv::Point(up,vp),2,cv::Scalar(255,0,0),2,8,0);
        cv::line(show_in,  cv::Point(up,vp),  cv::Point(uc,vc), cv::Scalar(0,255,0),1,8,0);
        (*it_out).dis_c = sgbm_roi.at<short>(vc,uc);

        it_out++;
      }
      else
      {
        it_out = vo.quadmatches_outlier.erase(it_out);
      }
      
    }

    else
    {
      it_out = vo.quadmatches_outlier.erase(it_out);
    }
    
  }
  
  /* show the inliers and outliers in one picture */
  if(true)
  {
     //cv::imshow("show_in",show_in);
     //cv::waitKey(0);

  }

  if(false)
  {
      cv::imshow("show_in",show_in);
      cv::waitKey(0);
  }

}




void UVDisparity::calUDisparity(const cv::Mat& img_dis, cv::Mat& xyz,cv::Mat& roi_mask,cv::Mat& ground_mask)
{
  double max_dis = 0.0;
  cv::minMaxIdx (img_dis,NULL,&max_dis,NULL,NULL);
  max_dis = max_dis/16;//the stereosgbm algorithm amplifies the real disparity value by 16
      
  int d_cols = img_dis.cols;int d_rows = img_dis.rows;
  int u_rows = cvCeil(max_dis)+1;int u_cols = img_dis.cols;
    
  //allocate the memory for v-disparity map and initialize it as all zeros
  u_dis_int = Mat::zeros(u_rows, u_cols, CV_32SC1);

    for(int i = 0;i<d_rows;i++)
    {
      //const uchar* gray_ptr = img_rgb.ptr<uchar>(i);
      const short* disp_ptr = img_dis.ptr<short>(i);
 
      for(int j = 0; j<d_cols; j++)
      {
        short d = disp_ptr[j];

        if(!cvIsInf(d) && !cvIsNaN(d) && d > 0)
        {
          int dis = cvRound(d/16);
          //set value for the udisparity map
          int* udis_ptr = u_dis_int.ptr<int>(dis);

          if(roi_mask.at<uchar>(i,j) > 0 && ground_mask.at<uchar>(i,j) > 0 && dis > 0)
          {
             udis_ptr[j]++;
          }

        }
        
      }
        
    }

   //assign to the uchar u-disparity map
    u_dis_.create(u_dis_int.size(),CV_8UC1);

    float scale = 255*1.0f/xyz.rows;

    for(int i = 0;i < u_rows;i++)
    {
        //const uchar* gray_ptr = img_rgb.ptr<uchar>(i);
        const int* u_ptr = u_dis_int.ptr<int>(i);
        uchar* u_char_ptr = u_dis_.ptr<uchar>(i);

        for(int j = 0; j < u_cols; j++)
        {
            int u = u_ptr[j];
            u_char_ptr[j] = u*scale;
        }
      }

    //assign for visualize
    cvtColor(u_dis_, u_dis_show, CV_GRAY2BGR);

//    cv::imshow("color",u_dis_);
//    cv::waitKey(0);

    int xyz_cols = xyz.cols;
    int xyz_rows = xyz.rows;

    for(int j = 0; j < xyz_rows; j++)
    {
        float* xyz_ptr = xyz.ptr<float>(j);
        for(int i = 0;i < xyz_cols; i++)
        {

             int u = cvRound(xyz_ptr[10*i+3]);
             int d = cvRound(xyz_ptr[10*i+5]);
             int u_i = u_dis_.at<uchar>(d,u);
             xyz_ptr[10*i+7] = u_i;
        }
    }
  // GaussianBlur(u_dis_,u_dis_,Size(3,3),0,0);
  //cv::imwrite("U_disparity.png",u_dis_);
}


void UVDisparity::calVDisparity(const cv::Mat& img_dis,cv::Mat& xyz)
{
  double max_dis = 0.0;
  cv::minMaxIdx (img_dis,NULL,&max_dis,NULL,NULL);
  //cout<<"the max disparity is: "<<max_dis/16<<endl;
  max_dis = max_dis/16;//the stereosgbm algorithm amplifies the real disparity value by 16
  //cout<<"the maxim disparity is: "<<max_dis<<endl;
    
  int d_cols = img_dis.cols;
  int d_rows = img_dis.rows;

  int v_cols = cvCeil(max_dis);
  int v_rows = img_dis.rows;

   
  //allocate the memory for v-disparity map and initialize it as all zeros
   v_dis_int = Mat::zeros(v_rows, v_cols, CV_32SC1);

  for(int i = 0;i<d_rows;i++)
    {
      //const uchar* gray_ptr = img_rgb.ptr<uchar>(i);
      const short* disp_ptr = img_dis.ptr<short>(i);
      int* vdis_ptr = v_dis_int.ptr<int>(i);
 
      for(int j = 0; j<d_cols; j++)
      {
        short d = disp_ptr[j];

        if(!cvIsInf(d) && !cvIsNaN(d) && d > 0)
        {
          int dis = cvRound(d/16.0f);
          int id = max(0,min(v_cols,dis));

          vdis_ptr[id]++;
        }
      }
    }


  int xyz_cols = xyz.cols;
  int xyz_rows = xyz.rows;

  float scale = 255*1.0f/xyz.cols;

  //assign the int matrix to uchar matrix for visualize
  v_dis_.create(v_dis_int.size(),CV_8UC1);
  for(int i = 0;i < v_rows;i++)
  {
      //const uchar* gray_ptr = img_rgb.ptr<uchar>(i);
      const int* v_ptr = v_dis_int.ptr<int>(i);
      uchar* v_char_ptr = v_dis_.ptr<uchar>(i);

      for(int j = 0; j < v_cols; j++)
      {
          int v = v_ptr[j];
          v_char_ptr[j] = v*scale;
      }
    }

  //assign for visualize
  cvtColor(v_dis_, v_dis_show, CV_GRAY2BGR);

//  cv::imshow("v_dis",v_dis_uchar);
//  cv::waitKey(0);

  //assign to xyz 10D matrix
  for(int j = 0; j < xyz_rows; j++)
  {
      float* xyz_ptr = xyz.ptr<float>(j);
      for(int i = 0;i < xyz_cols; i++)
      {

           int v = cvRound(xyz_ptr[10*i+4]);
           int d = cvRound(xyz_ptr[10*i+5]);

           if(d>0)
           {
             int v_i = v_dis_.at<int>(v,d);
             xyz_ptr[10*i+8] = (float)v_i;
           }
           else
           {
             xyz_ptr[10*i+8] = 0;
           }

      }
  }


}
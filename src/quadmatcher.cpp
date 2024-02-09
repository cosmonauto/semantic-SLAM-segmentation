
/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "quadmatcher.hpp"
#include <array>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core.hpp"


using namespace cv;
using namespace std;







QuadFeatureMatch::QuadFeatureMatch(cv::Mat& img_lc_,cv::Mat& img_rc_,
                                   cv::Mat& img_lp_, cv::Mat& img_rp_,
				   cv::Mat& img_s_rc_, cv::Mat& img_s_rp_,
                                   bool mode_track_)
{
   img_lc = img_lc_;
   img_rc = img_rc_;
   img_lp = img_lp_;
   img_rp = img_rp_;
   img_s_rc = img_s_rc_;
   img_s_rp = img_s_rp_;

   mode_track = mode_track_;
}


void QuadFeatureMatch::matching(vector<KeyPoint>& keypoints1, cv::Mat& descriptors1,
                                 vector<KeyPoint>& keypoints2, cv::Mat& descriptors2,
                                 int search_width, int search_height, vector<DMatch>& matches)
 {
     //double time_test1 = (double)cv::getTickCount();
     //int min_distance_int = 9999999;

     cv::Point2f pt1,pt2;
     int num1 = keypoints1.size();
     int num2 = keypoints2.size();
     const KeyPoint* ptr1 = (num1 != 0) ? &keypoints1.front() : 0;
     const KeyPoint* ptr2 = (num2 != 0) ? &keypoints2.front() : 0;

     for(int i=0;i<num1;i++)
     {
         pt1 = ptr1[i].pt;
         int id = 0;
         float min_distance = 999999999.9f;

         for(int j = 0; j<num2;j++)
         {
             pt2 = ptr2[j].pt;
             if(std::abs(pt2.x - pt1.x)<search_width && std::abs(pt2.y - pt1.y)<search_height)
             {
                 float distance = caldistance(descriptors1.row(i),descriptors2.row(j),descriptor_binary);
                 cout<<distance<<endl;
		 
                 if(distance<min_distance)
                 {
		     {
                         min_distance = distance;
                         id = j;
		     }
                 }
             }
         }

         if(min_distance > distance_threshold) id = -1;

         matches.push_back(DMatch(i,id,min_distance));
     }

 }




void QuadFeatureMatch::drawMatchesQuad(int time)
{
    cv::Mat img_quad(img_lc.rows*2,img_lc.cols*2,img_lc.type());

    img_lc.copyTo(img_quad(cv::Rect(0,0,img_lc.cols,img_lc.rows)));
    img_rc.copyTo(img_quad(cv::Rect(img_lc.cols,0,img_lc.cols,img_lc.rows)));
    img_lp.copyTo(img_quad(cv::Rect(0,img_lc.rows,img_lc.cols,img_lc.rows)));
    img_rp.copyTo(img_quad(cv::Rect(img_lc.cols,img_lc.rows,img_lc.cols,img_lc.rows)));

    cv::Mat img_quad_show;
    cvtColor(img_quad, img_quad_show, CV_GRAY2BGR);

    cv::Scalar red(0,0,255);
    cv::Scalar green(0,255,0);
    cv::Scalar blue(255,0,0);
    int num_matches = quadmatches.size();
    cv::Point points_lc, points_rc,points_lp,points_rp;

    for(int i = 0;i<num_matches;i++)
    {
        cv::Mat test;
        img_quad_show.copyTo(test);

        if(1)
        {
            points_lc.x = quadmatches[i].u1c;
            points_lc.y = quadmatches[i].v1c;

            points_rc.x = quadmatches[i].u2c;
            points_rc.y = quadmatches[i].v2c;

            points_lp.x = quadmatches[i].u1p;
            points_lp.y = quadmatches[i].v1p;

            points_rp.x = quadmatches[i].u1p;
            points_rp.y = quadmatches[i].v1p;

        }

        if(0)
        {
            points_lc.x = point_lc[i].x;
            points_lc.y = point_lc[i].y;

            points_rc.x = point_rc[i].x;
            points_rc.y = point_rc[i].y;

            points_lp.x = point_lp[i].x;
            points_lp.y = point_lp[i].y;

            points_rp.x = point_rp[i].x;
            points_rp.y = point_rp[i].y;
        }



        cv::circle(img_quad_show,points_lc,1,red,2,8,0);
        cv::circle(img_quad_show,points_rc+cv::Point(img_lc.cols,0),1,blue,2,8,0);

        cv::circle(img_quad_show,points_lp+cv::Point(0,img_lc.rows),1,red,2,8,0);
        cv::circle(img_quad_show,points_rp+cv::Point(img_lc.cols,img_lc.rows),1,blue,2,8,0);

        cv::line(img_quad_show,points_lc,points_rc+cv::Point(img_lc.cols,0),green,1,8,0);
        cv::line(img_quad_show,points_lc,points_lp+cv::Point(0,img_lc.rows),green,1,8,0);
        cv::line(img_quad_show,points_rc + cv::Point(img_lc.cols,0),
                     points_rp + cv::Point(img_lc.cols,img_lc.rows),green,1,8,0);
        cv::line(img_quad_show,points_lp+cv::Point(0,img_lc.rows),
                     points_rp+cv::Point(img_lc.cols,img_lc.rows),green,1,8,0);

    }

    cv::imshow("image quad",img_quad_show);
    cv::imwrite("quadmatch.jpg",img_quad_show);
    cv::waitKey(time);


}

 void QuadFeatureMatch::drawMatchesFlow(int time)
 {
     cv::Mat img_show;
     cvtColor(img_lc, img_show, CV_GRAY2BGR);

     cv::Scalar red(0,0,255);
     cv::Scalar green(0,255,0);
     cv::Scalar blue(255,0,0);
     int num = quadmatches.size();

     for(int i = 0; i < num; i++)
     {
          cv::Point pt;
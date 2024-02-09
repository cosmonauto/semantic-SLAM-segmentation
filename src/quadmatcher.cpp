
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
          pt.x = quadmatches[i].u1c;
          pt.y = quadmatches[i].v1c;

          cv::Point pt2;
          pt2.x = quadmatches[i].u1p;
          pt2.y = quadmatches[i].v1p;

          cv::circle(img_show,pt,1,red,2,8,0);
          cv::line(img_show,pt,pt2,green,1,8,0);
     }

     cv::imshow("image sparse flow",img_show);
     cv::waitKey(time);
 }

 void QuadFeatureMatch::drawMatchesSimple(int time)
 {
     cv::Mat img_show;
     cvtColor(img_lc, img_show, CV_GRAY2BGR);

     cv::Scalar red(0,0,255);
     cv::Scalar green(0,255,0);
     cv::Scalar blue(255,0,0);

     int num = point_lc.size();

     for(int i = 0; i < num; i++)
     {
         cv::Point2f pt = point_lc[i];
         cv::circle(img_show,pt,1,red,2,8,0);
     }

     cv::imshow("Detected Features",img_show);
     cv::waitKey(time);
 }





 void QuadFeatureMatch::init(int detector_type, int descriptor_type)
 {
    switch(detector_type)
     {
        case DET_FAST:
        {
            detector = FeatureDetector::create("FAST");
            detector->set("nonmaxSuppression", true);
            detector->set("threshold",20);
            break;
        }

        case DET_FAST_ADAPT:
        {
          detector = new DynamicAdaptedFeatureDetector (new FastAdjuster(30,true),800,1000,10);
          break;
        }
        case DET_FAST_GRID:
        {
         //detector = new GridAdaptedFeatureDetector(new FastFeatureDetector(10,true),800,10,10);

         detector = new GridAdaptedFeatureDetector(new DynamicAdaptedFeatureDetector (new FastAdjuster(20,true),12,15,10),
                                                   800,10,10);
          break;
        }
        case DET_STAR:
        {
            detector = FeatureDetector::create("STAR");
            detector->set("lineThresholdProjected",10);
            detector->set("lineThresholdBinarized",8);
            detector->set("suppressNonmaxSize",5);
            detector->set("responseThreshold",30);
            detector->set("maxSize", 16);
            break;
        }
       case DET_STAR_ADAPT:
       {
            detector = new DynamicAdaptedFeatureDetector (new StarAdjuster(40),800,1000,10);
            break;
       }

        case DET_STAR_GRID:
        {
         detector = new GridAdaptedFeatureDetector (new StarFeatureDetector(16,10,10,8,5),800,4,4);
            break;
        }

        case DET_ORB:
        {
            detector = FeatureDetector::create("ORB");
            //printParams(detector);
            detector->set("WTA_K",2);
            detector->set("scoreType",ORB::HARRIS_SCORE);
            detector->set("patchSize",31);
            detector->set("edgeThreshold",31);
            detector->set("scaleFactor", 1.2f);
            detector->set("nLevels", 5);
            detector->set("nFeatures",800);
            break;
        }
        case DET_SURF:
        {
            detector = FeatureDetector::create("SURF");
            //printParams(detector);
            detector->set("hessianThreshold", 400);
            detector->set("nOctaves",4);
            detector->set("nOctaveLayers",2);
            detector->set("upright",1);
            break;
        }
        case DET_SIFT:
        {
            detector = FeatureDetector::create("SIFT");
            //printParams(detector);
            detector->set("nFeatures", 1000);
            detector->set("nOctaveLayers",3);
            detector->set("contrastThreshold",0.04);
            detector->set("edgeThreshold",10);
            detector->set("sigma",1.6);
            break;
        }

        case DET_GFTT:
        {
             detector = FeatureDetector::create("GFTT");
             //printParams(detector);
             detector->set("qualityLevel",0.04);
             detector->set("minDistance",8);
            break;
        }

        case DET_GFTT_GRID:
        {
         detector = new GridAdaptedFeatureDetector (new GoodFeaturesToTrackDetector(20,0.01,5),800,8,8);
            //printParams(detector);
           // detector->set("qualityLevel",0.05);
           // detector->set("minDistance",5);
            break;
        }
     }

    if(mode_track == false)
     {
         switch(descriptor_type)
         {
            case DES_SIFT:
            {
                descriptor = DescriptorExtractor::create("SIFT");
                distance_threshold = 8000.0f;
                descriptor_binary = false;
                break;
            }
            case DES_SURF:
            {
                descriptor = DescriptorExtractor::create("SURF");
                distance_threshold = 0.3f;
                descriptor_binary = false;
                break;
            }
            case DES_BRISK:
            {
                descriptor = DescriptorExtractor::create("BRISK");
                distance_threshold = 120.0f;
                descriptor_binary = true;
                break;
            }
            case DES_FREAK:
            {
                descriptor = DescriptorExtractor::create("FREAK");
                distance_threshold = 100.0f;
                descriptor_binary = true;
                break;
            }
            case DES_ORB:
            {
                descriptor = DescriptorExtractor::create("ORB");
                distance_threshold = 80.0f;
                descriptor_binary = true;
                break;
            }
         }
     }

 }

void QuadFeatureMatch::extractDescriptor()
{

    if(!keypoint_lc.size() || !keypoint_lp.size()
            ||!keypoint_rc.size() || !keypoint_rp.size())
    {
        cout<<"Please Detect Feature Points At First!"<<endl;
        return;
    }

    {
            double time_descriptor = (double)cv::getTickCount();
            descriptor->compute( img_lc, keypoint_lc, descriptor_lc );
            descriptor->compute( img_rc, keypoint_rc, descriptor_rc );
            descriptor->compute( img_lp, keypoint_lp, descriptor_lp );
            descriptor->compute( img_rp, keypoint_rp, descriptor_rp );

            time_descriptor = ((double)cv::getTickCount() - time_descriptor)/cv::getTickFrequency()*1000;
            cout<<"The feature descriptor extraction in 4 images costs "<<time_descriptor<<" ms"<<endl;
     }
}



void QuadFeatureMatch::detectFeature()
{
    if(mode_track == false)
    {
        double time_detector = (double)cv::getTickCount();
        detector->detect(img_lc,keypoint_lc);
        detector->detect(img_rc,keypoint_rc);
        detector->detect(img_lp,keypoint_lp);
        detector->detect(img_rp,keypoint_rp);

        time_detector = ((double)cv::getTickCount() - time_detector)/cv::getTickFrequency()*1000;
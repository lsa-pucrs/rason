// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "jason_msgs/perception.h"

// c++ includes
#include <cstdlib>
#include <cstdio>
#include <fstream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/types_c.h>

// Boost includes
#include <boost/regex.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// others
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <zbar.h>  


#define CAMERA_FOV          57.0f  // kinect
//#define CAMERA_FOV          57.5f  // primesense

//#define CALIBRATION 1

#define BARCODES_DISTANCE    0.227f  // distance ( meters ) beetwen two barcodes from the same robot


// standart c++ namespace
using namespace std;

// OpenCV namespace
using namespace cv;
using namespace cv_bridge;
//using namespace cvb;

using namespace zbar;  


// ROS namespace
namespace enc = sensor_msgs::image_encodings;


//-----------------------------------------------------------------------------

class atsBlobFinder {
    
public:
    
    atsBlobFinder(cv::Mat src) {

        cv::Mat img; //must create a temporary Matrix to hold the gray scale or wont work
        cv::cvtColor( src, img, CV_BGR2GRAY ); //Convert image to GrayScale
        img = img > 1; //create the binary image
        ////cv::adaptiveThreshold(src,src,64,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,13); //create a binary image
        
        findContours( img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); //Find the Contour BLOBS
        const int nContoursSize = contours.size();
        
        vector<Moments> _mu( nContoursSize );
        vector<Point2f> _mc( nContoursSize );
        for( int i = 0; i < nContoursSize; i++ ) {

            _mu[ i ] = moments( Mat(contours[i]), false );
            _mc[ i ] = Point2f( _mu[i].m10 / _mu[i].m00 , _mu[i].m01 / _mu[i].m00);
        }
        mu = _mu;
        mc = _mc;
    }
    
    int GetBlobs( std::vector<Rect> &v ) {
        
        const int nSize = contours.size(); 
        int nRet = 0;
        
        {
            Rect r;
            for( int i = 0; i < nSize; i++ ) {

                r = cv::boundingRect( contours[ i ] );
                if ( r.width * r.height > 400 ) {

                    v.push_back( r );
                }
            }
        }
        
        if ( !v.empty() ) {
            
            int j, i = 0;
            bool bnHasIntersect;
            while( i < v.size() ) {
                
                j = i + 1;
                bnHasIntersect = false;
                
                while( j < v.size() ) {
                
                    Rect intersect = v[i] & v[j];
                    if ( intersect.height > 0 && intersect.width > 0 ) {

                        v[i] = v[i] | v[j];
                        v.erase( v.begin() + j );
                        bnHasIntersect = true;
                        continue;
                    }
                    j++;
                }
                if ( !bnHasIntersect ) {
                    i++;
                }
            }
            groupRectangles( v, 0, 1.0 );
            nRet = v.size();
        }
        return nRet;
    }
    
private:
    
    vector<vector<Point> > contours;
    vector<Vec4i>          hierarchy;
    vector<Moments>        mu;
    vector<Point2f>        mc;
};


struct ST_PerceptionData {
    
    Point2f         position;
    float           distance;
    float           yaw;
};

bool                                        g_bnHasDepth = false;
bool                                        g_bnHasRGB = false;
ros::Publisher                              g_oPerceptionPub;
pcl::PointCloud<pcl::PointXYZ> *            pcl_cloud;
Mat                                         g_oRGBFrame;
std::map< std::string, ST_PerceptionData *> g_mpstRobotData;

boost::mutex depth_mutex;
boost::mutex rgb_mutex;


#ifdef CALIBRATION
    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;
#endif

void RGBImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        
      cv_ptr = cv_bridge::toCvCopy( msg, enc::BGR8 );
    }
    catch( cv_bridge::Exception& e ) {
        
      ROS_ERROR( "cv_bridge exception: %s", e.what() );
      return;
    }
    
    rgb_mutex.lock();

    g_oRGBFrame = cv_ptr->image;
    g_bnHasRGB = true;
    
    rgb_mutex.unlock();
}

void DepthMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg ) {
    
    pcl::PCLPointCloud2    cloud;
    pcl_conversions::toPCL( *msg, cloud );

    pcl_cloud->clear();

    depth_mutex.lock();

    pcl::fromPCLPointCloud2( cloud, *pcl_cloud );
    g_bnHasDepth = true;

    depth_mutex.unlock();
}


void Perceive() {
    
    
    if ( !g_bnHasDepth || ! g_bnHasRGB ) {

            return;
    }
    
    std::map< std::string, ST_PerceptionData *> mpstComplexElement;
	
    jason_msgs::perception perception;
    
    Mat thresholded   = Mat( cvSize(640,480), CV_8UC1 );
    Mat thresholded2  = Mat( cvSize(640,480), CV_8UC3 );

    const Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 5, 5 ) );
    
    depth_mutex.lock();
    rgb_mutex.lock();
    
    cv::rectangle( g_oRGBFrame, 
        cvPoint( 319, 239 ),
        cvPoint( 321, 241 ),
        CV_RGB( 0, 255, 0 ), 1, 8, 0 );
    
    const float fCenterDepth = pcl_cloud->at( 320, 240 ).z;
    
    Mat hsv_frame;
    cv::cvtColor( g_oRGBFrame, hsv_frame, CV_BGR2HSV );
    
#ifdef CALIBRATION
    Mat imgThresholded;

    inRange(hsv_frame, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, element );
    dilate( imgThresholded, imgThresholded, element ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, element ); 
    erode(imgThresholded, imgThresholded, element );

    imshow("Thresholded Image", imgThresholded); //show the thresholded image
#endif
    
    // procedure to detect orange cylinder
    std::vector<Rect> vOrangeBlobs;
    cv::inRange( hsv_frame, Scalar( 0, 136, 159 ), Scalar( 29, 255, 255 ), thresholded );

    //morphological opening (remove small objects from the foreground)
    erode(thresholded, thresholded, element );
    dilate( thresholded, thresholded, element ); 

    //morphological closing (fill small holes in the foreground)
    dilate( thresholded, thresholded, element ); 
    erode(thresholded, thresholded, element );

    cv::cvtColor( thresholded, thresholded2, CV_GRAY2BGR, 3 );

    atsBlobFinder orange( thresholded2 );
    const int nOrangeSize = orange.GetBlobs( vOrangeBlobs );

    // procedure to detect green cylinder
    std::vector<Rect> vGreenBlobs;
    cv::inRange( hsv_frame, Scalar( 41, 81, 63 ), Scalar( 66, 215, 255 ), thresholded );

    //morphological opening (remove small objects from the foreground)
    erode(thresholded, thresholded, element );
    dilate( thresholded, thresholded, element ); 

    //morphological closing (fill small holes in the foreground)
    dilate( thresholded, thresholded, element ); 
    erode(thresholded, thresholded, element );

    cv::cvtColor( thresholded, thresholded2, CV_GRAY2BGR, 3 );

    atsBlobFinder green( thresholded2 );
    const int nGreenSize = green.GetBlobs( vGreenBlobs );

    int   nBestGreen    = -1;
    int   nBestOrange   = -1;
    
    const int nFrameWidth = g_oRGBFrame.cols;
    
    // identify valid blobs
    if ( !vGreenBlobs.empty() && !vOrangeBlobs.empty() ) {
        
        Point2f greenpt, orangept;
        
        float fBestDistance = cv::norm( Point2f( 0.0f, 0.0f ) - Point2f( g_oRGBFrame.cols, g_oRGBFrame.rows ) );
		
        for( int nGreen = 0; nGreen < nGreenSize; nGreen++ ) {

            greenpt.x = vGreenBlobs[ nGreen ].x + vGreenBlobs[ nGreen ].width  / 2.0f; 
            greenpt.y = vGreenBlobs[ nGreen ].y + vGreenBlobs[ nGreen ].height / 2.0f; 

            for( int nOrange = 0; nOrange < nOrangeSize; nOrange++ ) {
                
                orangept.x = vOrangeBlobs[ nOrange ].x + vOrangeBlobs[ nOrange ].width  / 2.0f;
                orangept.y = vOrangeBlobs[ nOrange ].y + vOrangeBlobs[ nOrange ].height / 2.0f; 

                const double fDistance = cv::norm( orangept - greenpt );
                if ( fDistance < fBestDistance ) {
                    
                    //pcl::PointXYZ pointO = pcl_cloud->at( orangept.x, orangept.y );
                    //pcl::PointXYZ pointG = pcl_cloud->at( greenpt.x , greenpt.y  );
                    
                    //if ( fabs( pointO.z - pointG.z ) <= BARCODES_DISTANCE ) {
                        
                        fBestDistance = fDistance;
                        nBestGreen    = nGreen;
                        nBestOrange   = nOrange;
                    //}
                }
            }
        }
        
        if ( nBestGreen > -1 && nBestOrange > -1 ) {
            
            greenpt.x = vGreenBlobs[ nBestGreen ].x + vGreenBlobs[ nBestGreen ].width  / 2.0f; 
            greenpt.y = vGreenBlobs[ nBestGreen ].y + vGreenBlobs[ nBestGreen ].height / 2.0f; 
            const float fGreenDepth = pcl_cloud->at( greenpt.x, greenpt.y ).z;
            
            cv::rectangle( g_oRGBFrame, 
                    vGreenBlobs[ nBestGreen ].tl(),
                    vGreenBlobs[ nBestGreen ].br(),
                    fGreenDepth != fGreenDepth ? CV_RGB( 0, 0, 255 ) : CV_RGB( 0, 255, 0 ),
                    2, 8, 0 );

            orangept.x = vOrangeBlobs[ nBestOrange ].x + vOrangeBlobs[ nBestOrange ].width  / 2.0f;
            orangept.y = vOrangeBlobs[ nBestOrange ].y + vOrangeBlobs[ nBestOrange ].height / 2.0f; 
            const float fOrangeDepth = pcl_cloud->at( orangept.x, orangept.y ).z;

            cv::rectangle( g_oRGBFrame,
                    vOrangeBlobs[ nBestOrange ].tl(),
                    vOrangeBlobs[ nBestOrange ].br(),
                    fOrangeDepth != fOrangeDepth ? CV_RGB( 0, 0, 255 ) : CV_RGB( 0, 255, 0 ),
                    2, 8, 0 );
            
            cv::line( g_oRGBFrame, orangept, greenpt, CV_RGB( 0, 255, 0 ), 1, 8, 0 );
            
            depth_mutex.unlock();
            cv::imshow( "imgwnd", g_oRGBFrame );
            rgb_mutex.unlock();
                
            if ( fGreenDepth != fGreenDepth || fOrangeDepth != fOrangeDepth ) {
                
                // I got a NaN !!!
                return;
            }

            const Point2f oCenter = ( orangept + greenpt ) * 0.5f;
            
            const float fAngle = ( ( oCenter.x - ( nFrameWidth * 0.5f ) ) / ( nFrameWidth * 0.5f ) ) * ( CAMERA_FOV / 2.0f );
            const float fDepth = ( fGreenDepth + fOrangeDepth ) / 2.0f;
            
            float fYaw = ( ( fGreenDepth - fOrangeDepth ) / BARCODES_DISTANCE ) * 90.f;
            if ( greenpt.x < orangept.x ) {
                
                if ( fOrangeDepth > fGreenDepth ) {
                    
                    fYaw += -90.0f;
                }
                else {
                    
                    fYaw = 180 - fYaw;
                }
            }

            ST_PerceptionData *poData;
            
            try {
               
                poData = g_mpstRobotData.at( "turtle1" );
            }
            catch( const std::out_of_range& ) {
               
                poData           = new ST_PerceptionData;
                poData->distance = 999.9f;
                poData->yaw      = 999.9f;
                g_mpstRobotData.insert( std::make_pair( ( std::string() + "turtle1" ), poData ) );
            }
            
            if ( cv::norm( poData->position - oCenter ) > 10.0 || 
                    fabs( poData->distance - fDepth ) > 0.05f ||
                    fabs( poData->yaw - fYaw ) > 7.0f ) {
               
                poData->position = oCenter;
                poData->distance = fDepth;
                poData->yaw      = fYaw;
               
                perception.perception.push_back( boost::str( boost::format( "robot(turtle1,%.0f,%.2f,%.0f)" ) 
                        % fAngle
                        % fDepth
                        % fYaw ) );
            }
        }
    }
    
    if ( nBestGreen == -1 || nBestOrange == -1 ) {
    
        depth_mutex.unlock();

        cv::imshow( "imgwnd", g_oRGBFrame );

        rgb_mutex.unlock();
    }
    
    ROS_INFO( "center depth = %f", fCenterDepth );
    
    if ( !perception.perception.empty() ) {
         
        g_oPerceptionPub.publish<jason_msgs::perception>( perception );
        BOOST_FOREACH( std::string strPerception, perception.perception ) {
            
            ROS_INFO( "%s", strPerception.c_str() );
        }
        ROS_INFO( " " );
    }
}


//-----------------------------------------------------------------------------

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "img_tracker");
    ros::NodeHandle n;

    pcl_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    
    ros::Subscriber subRGB = n.subscribe( "/camera/rgb/image_raw", 1, RGBImageCallback );
    ros::Subscriber subD   = n.subscribe( "/camera/depth_registered/points" , 1, DepthMapCallback );

    g_oPerceptionPub = n.advertise< jason_msgs::perception >( "/jason/perception", 5 );

    cv::namedWindow( "imgwnd" );
    cv::resizeWindow( "imgwnd", 320, 240 );
    
#ifdef CALIBRATION
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
#endif
    
    while( ros::ok() ) {
        
        ros::spinOnce();
        Perceive();
        
        cvWaitKey(1); 
    }

    return 0;	
}

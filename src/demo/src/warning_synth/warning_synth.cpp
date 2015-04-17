// c++ includes
#include <cstdlib>
#include <cstdio>
#include <fstream>

// Boost includes
#include <boost/format.hpp>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// project includes
#include "jason_msgs/perception.h"


//-----------------------------------------------------------------------------

int main(int argc, char **argv) {
    
    const std::string strNodeName( "warning_synth" );
    
    // initialize random seed
    srand( time( NULL ) );

    // node initialization
    ros::init( argc, argv, strNodeName );
    ros::NodeHandle n;
    jason_msgs::perception perception;
    perception.source = strNodeName;

    ros::Publisher oPerceptionPub = n.advertise< jason_msgs::perception >( "/jason/perception", 5 );
    
    time_t stLast, stCurrent;
    time( &stLast );
    
    // generate first timeout
    float fAlertTimeout = ( rand() % 1001 ) / 100.0f;
    ROS_INFO("Alert message in %.02f seconds", fAlertTimeout );

    while( ros::ok() ) {
        
        // check ROS system
        ros::spinOnce();
        
        // get current time and decrease elapsed time since last loop from remaining time
        time( &stCurrent );
        fAlertTimeout -= difftime( stCurrent, stLast );
        
        // if timeout, send an alert message
        if ( fAlertTimeout < 0.0f ) {
            
            const float fX = ( rand() % 501 ) / 100.0f;
            const float fY = ( rand() % 501 ) / 100.0f;

            std::string strPerception = boost::str( boost::format( "warning(%.02f,%.02f)" ) % fX % fY );
            
            // send perception to Jason by publishing on /jason/perception
            perception.perception.clear();
            perception.perception.push_back( strPerception );
            oPerceptionPub.publish<jason_msgs::perception>( perception );
			
            // generate new timeout
            fAlertTimeout = ( rand() % 2001 ) / 100.0f + 20.0f;
            ROS_INFO("Alert message in %.02f seconds", fAlertTimeout );
        }
		
        // update time info
        memcpy( &stLast, &stCurrent, sizeof( time_t ) );
    }

    return 0;	
}

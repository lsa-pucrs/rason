#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/format.hpp>
#include <string>

#include "jason_msgs/action.h"
#include "jason_msgs/event.h"
#include "JasonAction.h"
#include "JasonActionMove.h"
#include "JasonActionTurn.h"

MoveBaseClient *ac;
std::list<CJasonAction*> g_lpoAction;
        
void actionCallback( const jason_msgs::action::ConstPtr& msg ) {

    const std::string strAction( boost::to_upper_copy( msg->action ) );
    if ( strAction == "MOVE" ) {

        ROS_INFO( "Action %s queued", strAction.c_str() );
        g_lpoAction.push_back( new CJasonActionMove( *ac, msg ) );
    }
    else if ( strAction == "TURN" ) {

        ROS_INFO( "Action %s queued", strAction.c_str() );
        g_lpoAction.push_back( new CJasonActionTurn( *ac, msg ) );
    }
    else {
        
        ROS_INFO( "I don't decompose %s actions...", strAction.c_str() );
    }
}

int main(int argc, char **argv ) {

   ROS_INFO( "motion_decomp is running... supported actions are:" );
   ROS_INFO( "move(meters), move(x-coord,y-coord)" );
   ROS_INFO( "ready." );

  ros::init(argc, argv, "motion_decomp");
  
  ros::NodeHandle *poNode = new ros::NodeHandle;
  
  ac = new MoveBaseClient( "move_base", true );
  
  CJasonAction *poCurrAction = NULL;

  ros::Subscriber oActionSub = poNode->subscribe( "/jason/action", 10, actionCallback   );
  ros::Publisher oEventPub   = poNode->advertise<jason_msgs::event>( "/jason/event", 10 );

  ros::Rate r(10); // 10 hz
  while ( ros::ok() ) {
      
      ros::spinOnce();
      if ( poCurrAction ) {
          
          if ( poCurrAction->Done() ) {
              
              jason_msgs::event evtFeedback;
              
              evtFeedback.agent         = poCurrAction->getAgent();
              evtFeedback.event_type    = "action_feedback";
              evtFeedback.parameters.push_back( boost::str( boost::format( "%d" ) % poCurrAction->getInternalId() ) );
              evtFeedback.parameters.push_back( poCurrAction->Success() ? "OK" : "NOK" );
              
              oEventPub.publish( evtFeedback );
          
              delete poCurrAction;
              poCurrAction = NULL;
              ROS_INFO("Done.");
          }
          else {
              
              poCurrAction->Step();
          }
      }
      else if ( !g_lpoAction.empty() ) {
          
          poCurrAction = g_lpoAction.front();
          g_lpoAction.pop_front();
          poCurrAction->Start();
      }
      r.sleep();
  }
  delete poNode;
  delete ac;
  
  return 0;
}

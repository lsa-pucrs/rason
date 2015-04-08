#include "JasonActionMove.h"

CJasonActionMove::CJasonActionMove( MoveBaseClient &rac, const jason_msgs::action::ConstPtr& poActionMsg )
: CJasonAction( poActionMsg )
, ac( rac )
, m_bnDone( false )
, m_bnResult( false )
{
    goal.target_pose.header.stamp       = ros::Time::now();
    goal.target_pose.pose.position.x    = boost::lexical_cast< float >( poActionMsg->parameters[0] );
    goal.target_pose.pose.orientation.w = 1;
    if ( poActionMsg->parameters.size() > 1 ) {
        
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.y = boost::lexical_cast< float >( poActionMsg->parameters[1] );
        ROS_INFO("moving to %.02f,%.02f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y );
    }
    else {
        
        goal.target_pose.header.frame_id = "base_link";
        ROS_INFO("moving %.02f meters", goal.target_pose.pose.position.x );
    }
}

CJasonActionMove::~CJasonActionMove() {
    
}

void CJasonActionMove::Start() {
    
    ROS_INFO( "Action move started..." );
    ac.sendGoal( goal );
}

void CJasonActionMove::Step() {
    
    // non-blocking approach
    if ( !m_bnDone && ac.getState().isDone() ) {
	
        m_bnDone   = true;
        m_bnResult = ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED );
    }
}

const bool CJasonActionMove::Done() {
 
	return m_bnDone;
}

const bool CJasonActionMove::Success() {
 
	return m_bnResult;
}

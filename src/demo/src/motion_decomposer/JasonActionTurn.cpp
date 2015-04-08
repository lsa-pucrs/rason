#include <tf/transform_datatypes.h>

#include "JasonActionTurn.h"

#define DEG2RAD(x) (0.017453292519*x)

CJasonActionTurn::CJasonActionTurn( MoveBaseClient &rac, const jason_msgs::action::ConstPtr& poActionMsg )
: CJasonAction( poActionMsg )
, ac( rac )
, m_bnDone( false )
, m_bnResult( false )
{
    const int nDeg = ( boost::lexical_cast< int >( poActionMsg->parameters[0] ) % 360 );
    
    goal.target_pose.header.frame_id  = "base_link";
    goal.target_pose.header.stamp     = ros::Time::now();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw( DEG2RAD( nDeg ) );
}

CJasonActionTurn::~CJasonActionTurn() {
    
}

void CJasonActionTurn::Start() {

    ac.sendGoal( goal );
    ROS_INFO( "action move started..." );
}

void CJasonActionTurn::Step() {
    
    // non-blocking approach
    if ( !m_bnDone && ac.getState().isDone() ) {
	
        m_bnDone   = true;
        m_bnResult = ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED );
    }
}

const bool CJasonActionTurn::Done() {
 
	return m_bnDone;
}

const bool CJasonActionTurn::Success() {
 
	return m_bnResult;
}


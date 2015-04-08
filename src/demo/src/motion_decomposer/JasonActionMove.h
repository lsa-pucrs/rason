#ifndef __JASONACTIONMOVE_H__
#define __JASONACTIONMOVE_H__

#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>

#include <ros/ros.h>

#include "JasonAction.h"

class CJasonActionMove : public CJasonAction {
    
    protected:
        
        move_base_msgs::MoveBaseGoal    goal;
        MoveBaseClient &                ac;
        bool                            m_bnDone;
        bool                            m_bnResult;
        
    public:
        
        CJasonActionMove( MoveBaseClient &, const jason_msgs::action::ConstPtr& );
        virtual ~CJasonActionMove();
        
        virtual void    		Step        ( void );
        virtual void    		Start       ( void );
        virtual const bool		Done		( void );
        virtual const bool      Success     ( void );
};


#endif //__JASONACTIONMOVE_H__

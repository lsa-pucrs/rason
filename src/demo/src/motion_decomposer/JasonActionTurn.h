#ifndef __JASONACTIONTURN_H__
#define __JASONACTIONTURN_H__

#include <ros/ros.h>

#include "JasonAction.h"

class CJasonActionTurn : public CJasonAction {

    protected:
        
        move_base_msgs::MoveBaseGoal    goal;
        MoveBaseClient &                ac;
        bool                            m_bnDone;
        bool                            m_bnResult;
        
    public:
        
        CJasonActionTurn( MoveBaseClient &, const jason_msgs::action::ConstPtr& );
        virtual ~CJasonActionTurn();
        
        virtual void        Step        ( void );
        virtual void        Start       ( void );
        virtual const bool  Done        ( void );
        virtual const bool  Success     ( void );
};


#endif //__JASONACTIONTURN_H__

#ifndef __ACTION_H__
#define __ACTION_H__

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "jason_msgs/action.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CJasonAction {
    
    protected:
    
        std::string     m_strAgent;
        unsigned int    m_wwInternalID;
        
    public:
    
        CJasonAction( const jason_msgs::action::ConstPtr& poAction )
        : m_strAgent( poAction->agent )
        , m_wwInternalID( poAction->action_id )
        {
            
        }
        
        std::string const &     getAgent        ( void ) { return m_strAgent; };
        const unsigned int      getInternalId   ( void ) { return m_wwInternalID; };
        
    
        virtual void    		Step            ( void ) = 0;
        virtual void    		Start           ( void ) = 0;
        virtual const bool		Done            ( void ) = 0;
        virtual const bool		Success         ( void ) = 0;
};

#endif //__ACTION_H__

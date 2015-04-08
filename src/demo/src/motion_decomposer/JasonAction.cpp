#include "JasonAction.h"

CJasonAction::CJasonAction()
: m_bnDone( false )
{
}

const bool CJasonAction::Done() {

    return m_bnDone;
}

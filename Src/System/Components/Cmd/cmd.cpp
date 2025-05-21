#include "cmd.hpp"

void Cmd::parse()
{
    RC::rc_ctrl_t rcData = rc_.getData();

    if (rcData.rc.switch_right == RC_SW_DOWN) {
        msg.state = State_e::stop;
    }
}

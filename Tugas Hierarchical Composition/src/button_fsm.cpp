#include "button_fsm.h"

void button_fsm :: setThresOne(int val) {
    count_thres_1 = val;
    return;
}

void button_fsm :: setThresTwo(int val) {
    count_thres_2 = val;
    return;
}

void button_fsm :: setThresThree(int val) {
    count_thres_3 = val;
    return;
}

void button_fsm :: setThresFour(int val) {
    count_thres_4 = val;
    return;
}

States button_fsm :: getState(void) {
    return state;
}

int button_fsm :: getCount(void) {
    return count;
}

output_struct button_fsm :: process(int input) {
    output_struct retval = {0,0,0};

    if (state == STATE_0) {
        if (input) {
            count = 0;
            state = STATE_1;
        }
    }

    else if (state == STATE_1) {
        if (count < count_thres_1) {
            count++;
        }
        else {
            count = 0;
            state = STATE_2;
        }
    }

    else if (state == STATE_2) {
        if (!input) {
            retval.single = 1;
            count = 0;
            state = STATE_6;
        }
        else {
            count = 0;
            state = STATE_3;
        }
    }

    else if (state == STATE_3) {
        if (!input) {
            retval.single = 1;
            count = 0;
            state = STATE_6;
        }
        else {
            if (count < count_thres_2) {
                count++;
            }
            else {
                count = 0;
                state = STATE_4;
            }
        }
    }

    else if (state == STATE_4) {
        if (!input) {
            retval.single = 1;
            count = 0;
            state = STATE_6;
        }
        else {
            if ((count%count_thres_3 + 1) == 0) {
                retval.auto_repeat = 1;
            }
            if (count < count_thres_4) {
                count++;
            }
            else {
                retval.hold = 1;
                count = count % count_thres_3 + 1;
                state = STATE_5;
            }
        }
    }

    else if (state == STATE_5) {
        if (!input) {
            count = 0;
            state = STATE_6;
        }
        else {
            if (count < count_thres_3) {
                count++;
            }
            else {
                retval.auto_repeat = 1;
                count = 0;
            }
        }
    }

    else if (state == STATE_6) {
        if (count < count_thres_1) {
            count++;
        }
        else {
            count = 0;
            state = STATE_0;
        }
    }

    return retval;
}
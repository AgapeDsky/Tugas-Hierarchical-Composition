#include "device_fsm.h"

void device_fsm :: updateParams(void) {
    a = (4*kd+2*sampling_time*kp+sampling_time*sampling_time*ki)/(2*sampling_time);
    b = (2*sampling_time*sampling_time*ki-8*kd)/(2*sampling_time);
    c = (4*kd-2*sampling_time*kp+sampling_time*sampling_time*ki)/(2*sampling_time);
    d = 1.;
    return;
}

void device_fsm :: updateSetPoint(float val) {
    velocity = val;
    prev_prev_err = prev_err;
    prev_err = err;
    err = target-velocity;
    return;
}

void device_fsm :: updateEffort(float val) {
    prev_prev_effort = prev_effort;
    prev_effort = effort;
    effort = val;
    return;
}

void device_fsm :: updateTarget(float val) {
    target = val;
    return;
}

void device_fsm :: clear(void) {
    velocity = 0;
    err = 0; prev_err = 0; prev_prev_err = 0;
    effort = 0; prev_effort = 0; prev_prev_effort = 0;
    return;
}

float device_fsm :: getKp(void) {
    return kp;
}

float device_fsm :: getKi(void) {
    return ki;
}

float device_fsm :: getKd(void) {
    return kd;
}

void device_fsm :: setKp(float val) {
    kp = val;
    return;
}

void device_fsm :: setKi(float val) {
    ki = val;
    return;
}

void device_fsm :: setKd(float val) {
    kd = val;
    return;
}

float device_fsm :: getTarget(void) {
    return target;
}

SuperState device_fsm :: getSuperState(void) {
    return super_state;
}

OperationalState device_fsm :: getOperationalState(void) {
    return operational_state;
}

SetState device_fsm :: getSetState(void) {
    return set_state;
}

output device_fsm :: process(int plus, int minus, int set, int hold, float vel) {
    float retval = 0;

    if (super_state == SET) {
        if (set_state == KP) {
            if (hold) {
                super_state = OPERATIONAL;
                operational_state = STANDBY;
                updateParams();
                retval = target;
            }
            else {
                if (set) {
                    set_state = KI;
                    retval = ki;
                }
                else if (plus) {
                    kp = (kp >= 9.8) ? kp : kp + 0.5;
                    retval = kp;
                }
                else if (minus) {
                    kp = (kp <= 0.2) ? kp : kp - 0.5;
                    retval = kp;
                }
                else {
                    retval = kp;
                }
            }
        }
        else if (set_state == KI) {
            if (hold) {
                super_state = OPERATIONAL;
                operational_state = STANDBY;
                updateParams();
                retval = target;
            }
            else {
                if (set) {
                    set_state = KD;
                    retval = kd;
                }
                else if (plus) {
                    ki = (ki >= 9.8) ? ki : ki + 0.5;
                    retval = ki;
                }
                else if (minus) {
                    ki = (ki <= 0.2) ? ki : ki - 0.5;
                    retval = ki;
                }
                else {
                    retval = ki;
                }
            }
        }
        else if (set_state == KD) {
            if (hold) {
                super_state = OPERATIONAL;
                operational_state = STANDBY;
                updateParams();
                retval = target;
            }
            else {
                if (set) {
                    set_state = KP;
                    retval = kp;
                }
                else if (plus) {
                    kd = (kd >= 9.8) ? kd : kd + 0.5;
                    retval = kd;
                }
                else if (minus) {
                    kd = (kd <= 0.2) ? kd : kd - 0.5;
                    retval = kd;
                }
                else {
                    retval = kd;
                }
            }
        }
    }
    else if (super_state == OPERATIONAL) {
        if (operational_state == STANDBY) {
            updateParams();
            if (hold) {
                super_state = SET;
                set_state = KP;
                clear();

                retval = kp;
            }
            else if (set) {
                operational_state = RUNNING;
                retval = effort;
            }
            else {
                retval = target;
            }
        }
        else if (operational_state == RUNNING) {
            if (hold) {
                super_state = SET;
                set_state = KP;
                clear();
                retval = kp;
            }
            else if (set) {
                operational_state = STANDBY;
                clear();
                retval = target;
            }
            else {
                prev_prev_err = prev_err;
                prev_err = err;
                prev_prev_effort = prev_effort;
                prev_effort = effort;
                velocity = vel;

                err = target - velocity;
                effort = a*err+b*prev_err+c*prev_prev_err+d*prev_prev_effort;

                retval = effort;
            }
        }
    }

    return {super_state, operational_state, set_state, retval};
}
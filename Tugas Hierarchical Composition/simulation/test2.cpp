#include "../src/device_fsm.h"
#include <stdio.h>

#define tau 1.2
#define T 0.008

float compute_plant(float effort_1, float effort, float output) {
    return effort_1/(2*tau+T)*T + effort/(2*tau+T)*T - output*(T-2*tau)/(2*tau+T);
};


int main() {
    device_fsm fsm;
    fsm.updateTarget(10);

    output effort;// = fsm.process(0,0,1,0,0);
    // effort = fsm.process(0,0,0,0,0);

    int plus, minus, set, hold;
    scanf("%d %d %d %d", &plus, &minus, &set, &hold);

    float effort_1 = 0, actuate = 0, actuate_1 = 0;

    effort = fsm.process(plus,minus,set,hold, 0);
    actuate = compute_plant(effort_1, effort.val, actuate_1);
    // printf("Super:%d  Op:%d  Set:%d  Val:%f\n", effort.super_state, effort.operational_state, effort.set_state, effort.val);

    while(1) {
        
       scanf("%d %d %d %d", &plus, &minus, &set, &hold);
       actuate_1 = actuate;
       effort_1 = effort.val;
       effort = fsm.process(plus,minus,set,hold,actuate);
       actuate = compute_plant(effort_1, effort.val, actuate_1);
    //    printf("effort:%f output:%f\n", effort, actuate);
    }
}
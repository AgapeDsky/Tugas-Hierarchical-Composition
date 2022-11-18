#include "../src/device_fsm.h"
#include <stdio.h>

#define tau 1.2
#define T 0.008

float compute_plant(float effort_1, float effort, float output) {
    return effort_1/(2*tau+T)*T + effort/(2*tau+T)*T - output*(T-2*tau)/(2*tau+T);
};


int main() {
    device_fsm fsm;

    output effort;

    while(1) {
        int plus, minus, set, hold;
        scanf("%d %d %d %d", &plus, &minus, &set, &hold);

        effort = fsm.process(plus, minus, set, hold, 0);

        printf("SuperState:%d   OperationalState:%d   SetState:%d   Value:%f\n", effort.super_state, effort.operational_state, effort.set_state, effort.val);
    }
}
#include <stdio.h>
#include "../src/button_fsm.h"

int main() {
    button_fsm fsm;
    output_struct output = {0,0,0};

    fsm.setThresOne(2);
    fsm.setThresTwo(2);
    fsm.setThresThree(2);
    fsm.setThresFour(2);

    while (1) {
        int input = 0;
        scanf("%d", &input);
        output = fsm.process(input);

        printf("single: %d  hold: %d  auto: %d  state: %d  count: %d\n", output.single, output.hold, output.auto_repeat, fsm.getState(), fsm.getCount());
    }
    return 0;
}
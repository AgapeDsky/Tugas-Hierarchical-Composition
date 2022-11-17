#ifndef BUTTON_FSM_H
#define BUTTON_FSM_H

/**
 * @brief States representing button fsm conditions 
 */
enum States {
    STATE_0,
    STATE_1,
    STATE_2,
    STATE_3,
    STATE_4,
    STATE_5,
    STATE_6
};

/**
 * @brief Output struct from fsm's process
 * @param single single command
 * @param hold hold command
 * @param auto_repeat autorepeat command 
 */
typedef struct output_struct {
    int single;
    int hold;
    int auto_repeat;
} output_struct;

/**
 * @brief fsm to detect single command, hold command, and auto repeat command from buttons
 * @brief built using extended fsm
 * @brief computational output is passed to other system in a cascaded manner
 */
class button_fsm {
    private:
        int count = 0;              // Counter
        int count_thres_1 = 100;    // Threshold for debouncing
        int count_thres_2 = 100;    // Threshold for single detection
        int count_thres_3 = 100;    // Threshold for autorepeat detection
        int count_thres_4 = 100;    // Threshold for hold detection
        States state = STATE_0;     // fsm's state
        
    public:
        /**
         * @brief compute fsm process
         * 
         * @param input input command from buttons
         * @return output_struct single, hold, and autorepeat command
         */
        output_struct process(int input);

        /**
         * @brief Set the count_thres_1 object
         * 
         * @param val threshold
         */
        void setThresOne(int val);

        /**
         * @brief Set the count_thres_2 object
         * 
         * @param val threshold
         */
        void setThresTwo(int val);

        /**
         * @brief Set the count_thres_3 object
         * 
         * @param val threshold
         */
        void setThresThree(int val);

        /**
         * @brief Set the count_thres_4 object
         * 
         * @param val threshold
         */
        void setThresFour(int val);

        /**
         * @brief Get the fsm's state
         * 
         * @return States fsm state
         */
        States getState(void);

        /**
         * @brief Get the Count object
         * 
         * @return int counter value
         */
        int getCount(void);
};

#endif
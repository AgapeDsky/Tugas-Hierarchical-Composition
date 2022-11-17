#ifndef DEVICE_FSM_H
#define DEVICE_FSM_H

/**
 * @brief Super state of the PID controller machine 
 */
enum SuperState {
    OPERATIONAL,
    SET
};

/**
 * @brief Operational states, happening during OPERATIONAL SuperState 
 */
enum OperationalState {
    STANDBY,
    RUNNING
};

/**
 * @brief Setup states, happening during SET SuperState 
 */
enum SetState {
    KP,
    KI,
    KD
};

/**
 * @brief fsm's output struct
 * @param super_state super state of fsm
 * @param operational_state operational_state of fsm
 * @param set_state set_state of fsm
 * @param val output value, showing values based on the state
 */
typedef struct output {
    SuperState super_state;
    OperationalState operational_state;
    SetState set_state;
    float val;
} output;

/**
 * @brief controller fsm implementation 
 */
class device_fsm {
    private:
        float kp = 2;                                                   // Proportional coefficient
        float ki = 1;                                                   // Integral coefficient
        float kd = 0;                                                   // Derivative coefficient
        float target = 0;                                               // Target velocity
        float velocity = 0;                                             // Current velocity (feedback)
        float err = 0, prev_err = 0, prev_prev_err = 0;                 // error and its delayed versions
        float effort = 0, prev_effort = 0, prev_prev_effort = 0;        // control effort (input to plant) and its delayed versions
        float sampling_time = 0.008;                                    // sampling time

        SuperState super_state = OPERATIONAL;
        OperationalState operational_state = STANDBY;
        SetState set_state = KP;

        float a = (4*kd+2*sampling_time+sampling_time*sampling_time)/2*sampling_time;       // PID coefficient
        float b = (2*sampling_time*sampling_time*ki-8*kd)/2*sampling_time;                  // PID coefficient
        float c = (4*kd-2*sampling_time*kp+sampling_time*sampling_time)/2*sampling_time;    // PID coefficient
        float d = 1.;                                                                       // PID coefficient

    public:
        /**
         * @brief function to update PID coefficient (a,b,c,d) 
         */
        void updateParams(void);

        /**
         * @brief function to update error (setpoint to the controller)
         * 
         * @param val new set point
         */
        void updateSetPoint(float val);

        /**
         * @brief function to update control effort
         * 
         * @param val new control effort
         */
        void updateEffort(float val);

        /**
         * @brief function to update set point (target velocity)
         * 
         * @param val new target
         */
        void updateTarget(float val);

        /**
         * @brief function to clear control values 
         */
        void clear(void);

        /**
         * @brief Get the Super State object
         * 
         * @return SuperState 
         */
        SuperState getSuperState(void);

        /**
         * @brief Get the Operational State object
         * 
         * @return OperationalState 
         */
        OperationalState getOperationalState(void);

        /**
         * @brief Get the Set State object
         * 
         * @return SetState 
         */
        SetState getSetState(void);

        /**
         * @brief Get the Kp object
         * 
         * @return float 
         */
        float getKp(void);

        /**
         * @brief Get the Ki object
         * 
         * @return float 
         */
        float getKi(void);

        /**
         * @brief Get the Kd object
         * 
         * @return float 
         */
        float getKd(void);

        /**
         * @brief Set the Kp object
         * 
         * @param val 
         */
        void setKp(float val);

        /**
         * @brief Set the Ki object
         * 
         * @param val 
         */
        void setKi(float val);

        /**
         * @brief Set the Kd object
         * 
         * @param val 
         */
        void setKd(float val);

        /**
         * @brief Get the Target object
         * 
         * @return float 
         */
        float getTarget(void);

        /**
         * @brief Function to elaborate device's general behavior
         * 
         * @param plus Increase button
         * @param minus Decrease button
         * @param set Set button
         * @return output Process' output
         */
        output process(int plus, int minus, int set, int hold, float vel);
};

#endif
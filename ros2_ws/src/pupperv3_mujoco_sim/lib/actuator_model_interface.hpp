#ifndef H_ACTUATOR_MODEL_INTERFACE
#define H_ACTUATOR_MODEL_INTERFACE

typedef double num_t;

class ActuatorModelInterface
{
public:
    virtual double run(num_t kp,                // Nm/rad
                       num_t kd,                // Nm/(rad/s)
                       num_t position,          // rad
                       num_t position_target,   // rad
                       num_t velocity,          // rad/s
                       num_t velocity_target,   // rad/s
                       num_t feedforward_torque // Nm
                       ) = 0;                   // Nm
};

#endif
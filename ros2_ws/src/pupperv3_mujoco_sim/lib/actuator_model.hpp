#ifndef ACTUATOR_MODEL
#define ACTUATOR_MODEL

#include <cassert>
#include <algorithm>
#include <iostream>

#include "actuator_model_interface.hpp"

// #define ACTUATOR_MODEL_DEBUG

typedef double num_t;

struct ActuatorParams
{
    num_t kp, kd;                // Nm/rad, Nm/(rad/s)
    num_t bus_voltage;           // V
    num_t kt;                    // V/(rad/s) or Nm/A, careful about winding type!!
    num_t phase_resistance;      // Ohms, careful about winding type!
    num_t saturation_torque;     // Nm
    num_t software_torque_limit; // Nm

    ActuatorParams(num_t kp,
                   num_t kd,
                   num_t bus_voltage,
                   num_t kt,
                   num_t phase_resistance,
                   num_t saturation_torque,
                   num_t software_torque_limit) : kp(kp),
                                                  kd(kd),
                                                  bus_voltage(bus_voltage),
                                                  kt(kt),
                                                  phase_resistance(phase_resistance),
                                                  saturation_torque(saturation_torque),
                                                  software_torque_limit(software_torque_limit)
    {
        assert(kp >= 0);                    // TODO add max
        assert(kd >= 0);                    // TODO add max
        assert(bus_voltage >= 0);           // TODO add max
        assert(kt >= 0);                    // TODO add max
        assert(phase_resistance >= 0);      // TODO add max
        assert(saturation_torque >= 0);     // TODO add max
        assert(software_torque_limit >= 0); // TODO add max
    }
};

class PIDActuatorModel : public ActuatorModelInterface
{
public:
    PIDActuatorModel(ActuatorParams actuator_params) : params_(actuator_params)
    {
    }
    double run(num_t kp,                // Nm/rad
               num_t kd,                // Nm/(rad/s)
               num_t position,          // rad
               num_t position_target,   // rad
               num_t velocity,          // rad/s
               num_t velocity_target,   // rad/s
               num_t feedforward_torque // Nm
    )
    {
        params_.kp = kp;
        params_.kd = kd;
        return run(position, position_target, velocity, velocity_target, feedforward_torque);
    }
    double run(num_t position,          // rad
               num_t position_target,   // rad
               num_t velocity,          // rad/s
               num_t velocity_target,   // rad/s
               num_t feedforward_torque // Nm
    )
    {
        num_t torque_command = params_.kp * (position_target - position) +
                               params_.kd * (velocity_target - velocity) +
                               feedforward_torque;
        torque_command = std::clamp(torque_command, -params_.software_torque_limit, params_.software_torque_limit);
        num_t emf = std::abs(params_.kt * velocity);
        num_t max_current = (params_.bus_voltage - emf) / params_.phase_resistance;
        num_t emf_limited_torque = max_current * params_.kt;
#ifdef ACTUATOR_MODEL_DEBUG
        std::cout << "emf_limited_torque: " << emf_limited_torque << std::endl;
#endif
        torque_command = std::clamp(torque_command, -emf_limited_torque, emf_limited_torque);
        torque_command = std::clamp(torque_command, -params_.saturation_torque, params_.saturation_torque);

        return torque_command;
    }
    void set_kp(float kp)
    {
        params_.kp = kp;
    }
    void set_kd(float kd)
    {
        params_.kd = kd;
    }

private:
    ActuatorParams params_;
};

#endif
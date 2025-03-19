#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double input, double setpoint);
    void setTunings(double kp, double ki, double kd);
    void reset();

private:
    double kp, ki, kd;
    double prevError;
    double integral;
    unsigned long prevTime;
};

#endif

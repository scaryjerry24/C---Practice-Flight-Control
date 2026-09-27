// altitude_hold.cpp
// 1-D altitude hold for a quadcopter using a PID controller.
// Output is CSV (t, z, vz, thrust) so it can be plotted in MATLAB.

#include <iostream>
#include <algorithm>

struct PID {
    double kp, ki, kd;
    double integral = 0.0;
    double prevError = 0.0;

    double update(double error, double dt) {
        integral += error * dt;                        // I term: running sum
        double derivative = (error - prevError) / dt;  // D term: finite difference
        prevError = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

int main() {
    // Vehicle and sim constants
    const double m  = 2.3;     // mass (kg)
    const double g  = 9.81;    // gravity (m/s^2)
    const double dt = 0.01;    // time step (s)
    const double tEnd = 20.0;  // sim length (s)

    const double hoverThrust = m * g;              // thrust to hover (N)
    const double maxThrust   = 2.0 * hoverThrust;  // T/W = 2

    // State
    double z  = 0.0;   // altitude (m)
    double vz = 0.0;   // vertical velocity (m/s)
    const double target = 30.0;  // target altitude (m)

    PID pid{2.0, 0.1, 1.5};  // kp, ki, kd

    std::cout << "t,z,vz,thrust\n";  // CSV header

    const int steps = static_cast<int>(tEnd / dt);
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;

        // 1. Controller
        double error  = target - z;
        double thrust = hoverThrust + pid.update(error, dt);    // feedforward + PID
        thrust = std::max(0.0, std::min(thrust, maxThrust));    // motor limits

        // 2. Physics (semi-implicit Euler)
        double az = thrust / m - g;
        vz += az * dt;
        z  += vz * dt;

        // 3. Ground check
        if (z < 0.0) {
            z  = 0.0;
            vz = 0.0;
        }

        // 4. Output
        std::cout << t << "," << z << "," << vz << "," << thrust << "\n";
    }
    return 0;
}
#ifndef H_SIMPLEROBOT
#define H_SIMPLEROBOT

#define INTEGRATION_EULER       0
#define INTEGRATION_RUNGEKUTTA  1

#include <math.h>

/* Can only move forward and turn around on the spot */
class SimpleRobot {
public:
    struct {
        double x;
        double y;
        double theta;
    } state;

    struct {
        double forward;
        double angular;
    } velocity;

    int integration_method;

    SimpleRobot() {
        state.x = 0.0;
        state.y = 0.0;
        state.theta = 0.0;
        velocity.forward = 0.0;
        velocity.angular = 0.0;
        integration_method = INTEGRATION_EULER;
    }

    SimpleRobot(double x, double y, double theta) {
        state.x = x;
        state.y = y;
        state.theta = theta;
        velocity.forward = 0.0;
        velocity.angular = 0.0;
        integration_method = INTEGRATION_EULER;
    }

    /* Integrates the velocities one step forward */
    void step(double deltaTime, double forward_velocity, double angular_velocity) {
        velocity.forward = forward_velocity;
        velocity.angular = angular_velocity;
        double dx = 0.0;
        double dy = 0.0;
        double dtheta = 0.0;
        if (integration_method == INTEGRATION_EULER) {
            dx = velocity.forward * cos(state.theta);
            dy = velocity.forward * sin(state.theta);
            dtheta = velocity.angular;
        }
        else if (integration_method == INTEGRATION_RUNGEKUTTA) { 
            dx = velocity.forward * cos(state.theta + (velocity.angular * deltaTime) / 2);
            dy = velocity.forward * sin(state.theta + (velocity.angular * deltaTime) / 2);
            dtheta = velocity.angular;
        }
        state.x = state.x + dx * deltaTime;
        state.y = state.y + dy * deltaTime;
        state.theta = state.theta + dtheta * deltaTime;
    }

    void set(double x, double y, double theta) {
        state.x = x;
        state.y = y;
        state.theta = theta;
    }

    void reset() {
        state.x = 0.0;
        state.y = 0.0;
        state.theta = 0.0;
    }
};

#endif
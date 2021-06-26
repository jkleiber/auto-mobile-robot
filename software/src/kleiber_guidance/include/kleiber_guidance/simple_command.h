#ifndef SIMPLE_COMMAND_H
#define SIMPLE_COMMAND_H

struct SimpleCommand {
    SimpleCommand(){}
    SimpleCommand(double _vel, double _yaw, bool _enabled) : vel(_vel), yaw(_yaw), enabled(_enabled) {}

    double vel;
    double yaw;
    bool enabled;
};

#endif
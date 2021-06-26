#ifndef ACTION_H
#define ACTION_H

#include <chrono>

class Action
{
    public:
        Action(double timeout) : timeout_(timeout)
        {
            start_time_ = std::chrono::steady_clock::now();
            is_stopped_ = false;
            is_started_ = false;
        }

        // Action actions. Use these to do things with actions
        void update();
        void stop_action();
        void start_action();

        // Status
        bool is_started();
        bool is_stopped();

        // Required functions to be implemented by Actions
        virtual void start() = 0;
        virtual void run() = 0;
        virtual void stop() = 0;

        virtual ~Action();

    private:
        double timeout_;
        bool is_stopped_;
        bool is_started_;

        std::chrono::steady_clock::time_point start_time_;
};

#endif
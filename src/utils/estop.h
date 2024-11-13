#pragma once

#include <csignal>
#include <functional>
#include <vector>

class Estop {
protected:
    Estop() {
        initialize_estop_handler();
        estop_functions.push_back(std::bind(&Estop::estop, this, std::placeholders::_1));
    }

    virtual void estop(int sig) = 0;

public:
    static std::vector<std::function<void(int)>> estop_functions;

    static void estop_handler(int sig) {
        for (auto &f : estop_functions) {
            f(sig);
        }
        exit(sig);
    }

    static void initialize_estop_handler() {
        static bool is_initialized = false;
        if (!is_initialized) {
            struct sigaction sa;
            sa.sa_handler = estop_handler;
            sigemptyset(&sa.sa_mask);
            sa.sa_flags = 0;

            sigaction(SIGINT, &sa, nullptr);
            sigaction(SIGTERM, &sa, nullptr);

            is_initialized = true;
        }
    }
};

std::vector<std::function<void(int)>> Estop::estop_functions;

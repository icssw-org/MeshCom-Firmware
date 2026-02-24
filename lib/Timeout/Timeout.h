#pragma once
#include <Arduino.h>

class Timeout {
public:
    Timeout();

    // set the timer to `time_ms` and start ticking
    void start(uint32_t time_ms);

    // returns a single true when time runs out and then resets to `time_ms`.
    bool periodic(uint32_t time_ms);

    // pause and resume the timer
    void pause(void);
    void resume(void);

    // runs out the timer so `time_over()` returns true
    void expire(void);

    // returns whether the time ran out
    bool time_over(void);

    // returns whether the timer is paused
    bool is_paused(void);

    // returns the milliseconds until the timer runs out
    uint32_t time_left_ms(void);

private:
    uint32_t time_ms_;
    uint32_t reset_time_ms_;
    bool time_over_forced_;
    bool paused_;
};

#include "Timeout.h"

Timeout::Timeout()
    : time_ms_(0)
    , reset_time_ms_(0)
    , time_over_forced_(true)
    , paused_(false)
{
}

void Timeout::start(uint32_t time_ms)
{
    paused_ = false;
    time_ms_ = time_ms;
    if (time_ms == 0) {
        time_over_forced_ = true;
    } else {
        reset_time_ms_ = millis();
        time_over_forced_ = false;
    }
}

bool Timeout::periodic(uint32_t time_ms)
{
    bool result = time_over();
    if (result)
        start(time_ms);
    return result;
}

void Timeout::pause(void)
{
    time_ms_ = time_left_ms();
    paused_ = true;
}

void Timeout::resume(void)
{
    if (paused_) {
        start(time_ms_);
    }
}

void Timeout::expire(void)
{
    time_over_forced_ = true;
    paused_ = false;
}

bool Timeout::time_over(void)
{
    if (paused_) {
        return false;
    }
    bool result = (time_over_forced_ || (uint32_t)(millis() - reset_time_ms_) >= time_ms_);
    if (result) {
        // make sure to stay expired and not roll over again
        time_over_forced_ = true;
    }
    return result;
}

bool Timeout::is_paused(void)
{
    return paused_;
}

uint32_t Timeout::time_left_ms(void)
{
    if (paused_) {
        return time_ms_;
    }
    if (time_over()) {
        return 0;
    }
    return time_ms_ - (uint32_t)(millis() - reset_time_ms_);
}

#pragma once
#include "clk.h"
#include <Arduino.h>


class Klokke {
private:
    int64_t lastTime;
    int64_t startTime;

public:
    Klokke(int num);
    uint64_t getDT();
    uint64_t getTime();
    double convertTime();
};
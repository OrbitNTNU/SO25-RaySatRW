#include "clock.h"
Klokke::Klokke(int num) {
    lastTime = esp_clk_rtc_time();
}
uint64_t Klokke::getDT() {
    uint64_t newTime = esp_clk_rtc_time();
    uint64_t dt = newTime - lastTime;
    lastTime = newTime;
    return dt;
}
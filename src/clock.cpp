#include "clock.h"
#define CONVERT_US_TO_MIN 60000000;

Klokke::Klokke(int num) {
    startTime = esp_clk_rtc_time();
    lastTime = startTime;
}
uint64_t Klokke::getDT() {
    uint64_t newTime = esp_clk_rtc_time();
    uint64_t dt = newTime - lastTime;
    lastTime = newTime;
    return dt;
}

uint64_t Klokke::getTime() {
    return esp_clk_rtc_time() - startTime;
}

double Klokke::convertTime() {
    uint64_t naatid =  esp_clk_rtc_time();
    double minutter = (double) getTime()/CONVERT_US_TO_MIN;
    //Serial.printf("RTC clk %i minutter: %f\n", naatid, minutter);
    return minutter;
}
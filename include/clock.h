#include "clk.h"

class Klokke {
private:
    int64_t lastTime;
public:
    Klokke(int num);
    uint64_t getDT();
};
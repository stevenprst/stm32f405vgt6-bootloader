#include "mainloop.hpp"

// The three hooks are declared extern "C" in main.h and the HAL callback in the HAL CAN header, so these
// definitions keep C linkage.
void mainLoopJumpToAppIfRequested(void)
{
}

void mainLoopInit(void)
{
}

void mainLoopRun(void)
{
}


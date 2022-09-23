// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-

#include "panic.h"

#include <Arduino.h>

void panic0_(char const *file, int line)
{
    SerialUSB.println();
    SerialUSB.print("****************");
    SerialUSB.print("panic:");
    SerialUSB.print(file);
    SerialUSB.print(':');
    SerialUSB.print(line);
    SerialUSB.print("****************");
    SerialUSB.println();
    while (true)
        ;
}

#include "Algorithms_AM.h"

Algorithms_AM::Algorithms_AM()
{
    AM = new AttitudeMeasurement("usb:id:0073424514", "usb:id:1139993081", "COM10", 4.8e-2);
}

Algorithms_AM::~Algorithms_AM()
{
    delete AM;
}


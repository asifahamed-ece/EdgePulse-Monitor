#include "CurrentSensor.h"

static NullCurrentSensor gNull;
static CurrentSensor*    gSensor = &gNull;

void setCurrentSensor(CurrentSensor* s) {
    gSensor = s ? s : &gNull;
}

CurrentSensor* currentSensor() {
    return gSensor;
}

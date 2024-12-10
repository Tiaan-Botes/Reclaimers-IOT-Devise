#include "DS18B20.h"

namespace GAIT {

    DS18B20::DS18B20()
        : oneWire(32) {
    }

    float DS18B20::getTemperature() {
        DallasTemperature sensors(&oneWire);

        sensors.begin();
        sensors.requestTemperatures();

        return sensors.getTempCByIndex(0);
    }

} // namespace GAIT

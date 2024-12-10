/*

This demonstrates how to save the join information in to permanent memory
so that if the power fails, batteries run out or are changed, the rejoin
is more efficient & happens sooner due to the way that LoRaWAN secures
the join process - see the wiki for more details.

This is typically useful for devices that need more power than a battery
driven sensor - something like a air quality monitor or GPS based device that
is likely to use up it's power source resulting in loss of the session.

The relevant code is flagged with a ##### comment

Saving the entire session is possible but not demonstrated here - it has
implications for flash wearing and complications with which parts of the
session may have changed after an uplink. So it is assumed that the device
is going in to deep-sleep, as below, between normal uplinks.

Once you understand what happens, feel free to delete the comments and
Serial.prints - we promise the final result isn't that many lines.

*/

#if !defined(ESP32)
#pragma error("This is not the example your device is looking for - ESP32 only")
#endif

#include <Preferences.h>

RTC_DATA_ATTR uint16_t bootCount = 0;

#include "GPS.h"
#include "LoRaWAN.hpp"

#include <OneWire.h>
#include <DallasTemperature.h>

#include <CQRobotTDS.h>

#define ONE_WIRE_BUS 32  
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

static GAIT::LoRaWAN<RADIOLIB_LORA_MODULE> loRaWAN(RADIOLIB_LORA_REGION,
                                                   RADIOLIB_LORAWAN_JOIN_EUI,
                                                   RADIOLIB_LORAWAN_DEV_EUI,
                                                   (uint8_t[16]) {RADIOLIB_LORAWAN_APP_KEY},
                                                   (uint8_t[16]) {RADIOLIB_LORAWAN_NWK_KEY},
                                                   RADIOLIB_LORA_MODULE_BITMAP);

static GAIT::GPS gps(2, 9600, SERIAL_8N1, 16, 17);

// abbreviated version from the Arduino-ESP32 package, see
// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/deepsleep.html
// for the complete set of options
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(F("Wake from sleep"));
    } else {
        Serial.print(F("Wake not caused by deep sleep: "));
        Serial.println(wakeup_reason);
    }

    Serial.print(F("Boot count: "));
    Serial.println(++bootCount); // increment before printing
}

void gotoSleep(uint32_t seconds) {
    loRaWAN.goToSleep();
    gps.goToSleep();

    Serial.println("[APP] Go to sleep");

    esp_sleep_enable_timer_wakeup(seconds * 1000UL * 1000UL); // function uses uS
    esp_deep_sleep_start();

    Serial.println(F("\n\n### Sleep failed, delay of 5 minutes & then restart ###\n"));
    delay(5UL * 60UL * 1000UL);
    ESP.restart();
}

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;        // wait for serial to be initalised
    delay(2000); // give time to switch to the serial monitor

    print_wakeup_reason();

    Serial.println(F("\nSetup"));

    loRaWAN.setup(bootCount);

    loRaWAN.setDownlinkCB([](uint8_t fPort, uint8_t* downlinkPayload, std::size_t downlinkSize) {
        Serial.print(F("[APP] Payload: fPort="));
        Serial.print(fPort);
        Serial.print(", ");
        GAIT::arrayDump(downlinkPayload, downlinkSize);
    });

    Serial.println(F("[APP] Aquire data and construct LoRaWAN uplink"));

    gps.setup();
    sensors.begin();


    std::string uplinkPayload = RADIOLIB_LORAWAN_PAYLOAD;
    uint8_t fPort = 221;

    #define SENSOR_COUNT 5
    float temperature;
    int analogValue;
    float voltage;
    float pH;
    int tdsValue; 
    float tdsVoltage; 
    float tds;


    switch (bootCount % SENSOR_COUNT)
    {
    case 1:
        if (gps.isValid()) {
            fPort = 1; // 1 is location
            uplinkPayload = std::to_string(gps.getLatitude()) + "," + std::to_string(gps.getLongitude()) + "," +
                            std::to_string(gps.getAltitude()) + "," + std::to_string(gps.getHdop());
        }else { 
            fPort = 101;
            uplinkPayload = "GPS sensor not available"; 
        }
        break;
    case 2:
        sensors.requestTemperatures(); // Request temperature from DS18B20
        temperature = sensors.getTempCByIndex(0);
        if (temperature != DEVICE_DISCONNECTED_C) { // Check if the reading is valid
            fPort = 2; // Set port for temperature data
            uplinkPayload = std::to_string(temperature);
        }else { 
            fPort = 102; 
            uplinkPayload = "Temperature sensor not available";
        }
        break;
    case 3:
        analogValue = analogRead(34); // Read the analog value for pH
        voltage = analogValue * (3.3 / 4095.0); // Convert ADC value to voltage
        pH = 3.5 * voltage + 0.15; // Example conversion formula, adjust based on calibration
        if (pH >= 0.0 && pH <= 14.0) { // Check if pH value is within a realistic range
            fPort = 3; // Set port for pH data
            uplinkPayload = std::to_string(pH) +","+std::to_string(voltage); 
        }else { 
            fPort = 103; 
            uplinkPayload = "PH sensor not available";
        }
        break;
    case 4:
        tdsValue = analogRead(26); // Read the TDS sensor analog value
        tdsVoltage = tdsValue * (3.3 / 4095.0); // Convert to voltage
        // give a calibration value
        tds = (tdsVoltage / (1.0 + 0.02 * (temperature - 25.0))) * 133.42; // Conversion formula
        if (tds >= 0.0) { // Check if TDS value is non-negative
            fPort = 4; // TDS data
            uplinkPayload = std::to_string(tds); // Send TDS in ppm
        }
        else{
            fPort = 104; // TDS data
            uplinkPayload = "TDS sensor not available";
        }
        break;
    }

    loRaWAN.setUplinkPayload(fPort, uplinkPayload);
}


void loop() {
    loRaWAN.loop();
}

// Does it respond to a UBX-MON-VER request?
// uint8_t ubx_mon_ver[] = { 0xB5,0x62,0x0A,0x04,0x00,0x00,0x0E,0x34 };

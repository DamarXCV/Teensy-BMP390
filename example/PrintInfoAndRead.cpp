#include <Arduino.h>
#include <BMP390.h>
#include <SPI.h>

constexpr uint16_t LOOP_HZ = 1;

BMP390 baro(&SPI, SPISettings(8'000'000, MSBFIRST, SPI_MODE0), 10);

// constexpr uint8_t INTERRUPT_PIN = 6;
// void onInterrupt() {
//     Serial.println("Interrupt received!");
// }

void setup()
{
    // Setup blink led
    pinMode(LED_BUILTIN, OUTPUT);

    // pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onInterrupt, CHANGE);

    // Setup USB Serial
    Serial.begin(115'200);
    while (!Serial) {
        delayMicroseconds(4);
    }

    // Init the SPI bus
    SPI.begin();

    // Init the BMP390 bus
    baro.begin();
    delayMicroseconds(1'000);

    //////////////////////////////////////////
    // Change settings
    //////////////////////////////////////////
    baro.setSPIMode(false);

    baro.setTemperatureSensorStatus(true);
    baro.setPressureSensorStatus(true);

    baro.setPressureOversampling(OsrOsr::X8);
    baro.setTemperatureOversampling(OsrOsr::NONE);
    baro.setIIRFilter(ConfigIRRFilter::Coefficient_3);
    baro.setOutputDataRate(OdrOdrSel::Prescaler4);

    baro.setDataReadyInterruptStatus(true);
    baro.setInterruptLevel(true);

    baro.setPowerMode(PwrCtrlMode::normal);

    //////////////////////////////////////////
    // Print info
    //////////////////////////////////////////
    Serial.print("ChipId: ");
    Serial.println(baro.getChipId());

    Serial.print("RevId: ");
    Serial.println(baro.getRevId());

    Serial.print("SPI Mode: ");
    Serial.println(baro.getSPIMode());

    Serial.print("PowerMode: ");
    Serial.println(baro.getPowerMode());

    Serial.print("Is Pressure enabled: ");
    Serial.println(baro.getPressureSensorStatus());

    Serial.print("Is Temperature enabled: ");
    Serial.println(baro.getTemperatureSensorStatus());

    Serial.print("Pressure Oversampling: ");
    Serial.println(baro.getPressureOversampling());

    Serial.print("Temperature Oversampling: ");
    Serial.println(baro.getTemperatureOversampling());

    Serial.print("Output Data Rate: ");
    Serial.println(baro.getOutputDataRate());

    Serial.print("IIR Filter: ");
    Serial.println(baro.getIIRFilter());

    Serial.print("Data Ready Interrupt Status: ");
    Serial.println(baro.getDataReadyInterruptStatus());
}

uint8_t ledState = false;
uint64_t iLoop = 0;

void loop()
{
    uint32_t loopCycleStart = micros();

    // Blink
    if (iLoop % LOOP_HZ == 0) {
        digitalWrite(LED_BUILTIN, ledState ^= 1);
    }

    // Print data
    Serial.println("#########################");
    Serial.print("is Temperature Data Available: ");
    Serial.println(baro.isTemperatureDataAvailable());
    Serial.print("is Pressure Data Available: ");
    Serial.println(baro.isPressureDataAvailable());
    Serial.print("Temperature in °C: ");
    Serial.println(baro.readCompensatedTemperature());
    Serial.print("Pressure in P: ");
    Serial.println(baro.readCompensatedPressure());
    Serial.print("Sensortime: ");
    Serial.println(baro.readSensorTime());

    ++iLoop;
    delayMicroseconds((1'000'000 / LOOP_HZ) - (micros() - loopCycleStart));
}

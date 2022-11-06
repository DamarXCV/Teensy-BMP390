#include <Arduino.h>
#include <unity.h>
#include <SPI.h>

#include <BMP390.h>

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

void testGetChipId(void)
{
    BMP390 sensor(SPISettings(8'000'000, MSBFIRST, SPI_MODE0), 10);
    sensor.begin(); 

    TEST_ASSERT_EQUAL(96, sensor.getChipId());
}

void testGetRevId(void)
{
    BMP390 sensor(SPISettings(8'000'000, MSBFIRST, SPI_MODE0), 10);
    sensor.begin();

    TEST_ASSERT_EQUAL(1, sensor.getRevId());
}

void testSPIMode(void)
{
    BMP390 sensor(SPISettings(8'000'000, MSBFIRST, SPI_MODE0), 10);
    sensor.begin(); 

    bool mode = sensor.getSPIMode();
    sensor.setSPIMode(mode);
    TEST_ASSERT_EQUAL(mode, sensor.getSPIMode());
}

void testPowerMode(void)
{
    BMP390 sensor(SPISettings(8'000'000, MSBFIRST, SPI_MODE0), 10);
    sensor.begin(); 

    PwrCtrlMode mode = sensor.getPowerMode();
    bool pressureEnabled = sensor.getPressureSensorStatus();
    bool temperatureEnabled = sensor.getTemperatureSensorStatus();
    OsrOsr pressureOSR = sensor.getPressureOversampling();
    OsrOsr temperatureOSR = sensor.getTemperatureOversampling();

    sensor.setPressureSensorStatus(true);
    sensor.setTemperatureSensorStatus(true);
    sensor.setPressureOversampling(OsrOsr::NONE);
    sensor.setTemperatureOversampling(OsrOsr::NONE);

    sensor.setPowerMode(PwrCtrlMode::sleep);
    TEST_ASSERT_EQUAL(PwrCtrlMode::sleep, sensor.getPowerMode());

    delayMicroseconds(100);

    sensor.setPowerMode(PwrCtrlMode::forced);
    TEST_ASSERT_EQUAL(PwrCtrlMode::forced, sensor.getPowerMode());

    delayMicroseconds(5000);

    TEST_ASSERT_EQUAL(PwrCtrlMode::sleep, sensor.getPowerMode());

    delayMicroseconds(100);

    sensor.setPowerMode(PwrCtrlMode::normal);
    TEST_ASSERT_EQUAL(PwrCtrlMode::normal, sensor.getPowerMode());

    delayMicroseconds(100);

    sensor.setPowerMode(PwrCtrlMode::sleep);
    TEST_ASSERT_EQUAL(PwrCtrlMode::sleep, sensor.getPowerMode());

    delayMicroseconds(100);
    
    sensor.setPressureSensorStatus(pressureEnabled);
    sensor.setTemperatureSensorStatus(temperatureEnabled);
    sensor.setPressureOversampling(pressureOSR);
    sensor.setTemperatureOversampling(temperatureOSR);

    delayMicroseconds(100);

    sensor.setPowerMode(mode);
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    SPI.begin(); 

    UNITY_BEGIN(); // IMPORTANT LINE!

    RUN_TEST(testGetChipId);
    delay(100);
    RUN_TEST(testGetRevId);
    delay(100);
    RUN_TEST(testSPIMode);
    delay(100);
    RUN_TEST(testPowerMode);
    delay(100);
    // RUN_TEST(testGetRevId);
    // delay(100);
    // RUN_TEST(testGetRevId);
    // delay(100);
    // RUN_TEST(testGetRevId);
    // delay(100);
    
    UNITY_END(); // stop unit testing
}

void loop() { }
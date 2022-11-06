#pragma once

#include <SPIDevice.h>
#include <stdint.h>

//////////////////////////////////////////
// Register defines
//////////////////////////////////////////
constexpr uint8_t REGISTER_CHIP_ID = 0x00;
constexpr uint8_t REGISTER_REV_ID = 0x01;
constexpr uint8_t REGISTER_ERR_REG = 0x02;
constexpr uint8_t REGISTER_STATUS = 0x03;
constexpr uint8_t REGISTER_DATA_0 = 0x04;
constexpr uint8_t REGISTER_DATA_1 = 0x05;
constexpr uint8_t REGISTER_DATA_2 = 0x06;
constexpr uint8_t REGISTER_DATA_3 = 0x07;
constexpr uint8_t REGISTER_DATA_4 = 0x08;
constexpr uint8_t REGISTER_DATA_5 = 0x09;
// 0x0A - 0x0B reserved
constexpr uint8_t REGISTER_SENSORTIME_0 = 0x0C;
constexpr uint8_t REGISTER_SENSORTIME_1 = 0x0D;
constexpr uint8_t REGISTER_SENSORTIME_2 = 0x0E;
// 0x0F reserved
constexpr uint8_t REGISTER_EVENT = 0x10;
constexpr uint8_t REGISTER_INT_STATUS = 0x11;
constexpr uint8_t REGISTER_FIFO_LENGTH_0 = 0x12;
constexpr uint8_t REGISTER_FIFO_LENGTH_1 = 0x13;
constexpr uint8_t REGISTER_FIFO_DATA = 0x14;
constexpr uint8_t REGISTER_FIFO_WTM_0 = 0x15;
constexpr uint8_t REGISTER_FIFO_WTM_1 = 0x16;
constexpr uint8_t REGISTER_FIFO_CONFIG_1 = 0x17;
constexpr uint8_t REGISTER_FIFO_CONFIG_2 = 0x18;
constexpr uint8_t REGISTER_INT_CTRL = 0x19;
constexpr uint8_t REGISTER_IF_CONF = 0x1A;
constexpr uint8_t REGISTER_PWR_CTRL = 0x1B;
constexpr uint8_t REGISTER_OSR = 0x1C;
constexpr uint8_t REGISTER_ODR = 0x1D;
// 0x1E reserved
constexpr uint8_t REGISTER_CONFIG = 0x1F;
// 0x20 - 0x30 reserved
constexpr uint8_t REGISTER_NVM_PAR_T1_LSB = 0x31;
constexpr uint8_t REGISTER_NVM_PAR_T1_MSB = 0x32;
constexpr uint8_t REGISTER_NVM_PAR_T2_LSB = 0x33;
constexpr uint8_t REGISTER_NVM_PAR_T2_MSB = 0x34;
constexpr uint8_t REGISTER_NVM_PAR_T3 = 0x35;
constexpr uint8_t REGISTER_NVM_PAR_P1_LSB = 0x36;
constexpr uint8_t REGISTER_NVM_PAR_P1_MSB = 0x37;
constexpr uint8_t REGISTER_NVM_PAR_P2_LSB = 0x38;
constexpr uint8_t REGISTER_NVM_PAR_P2_MSB = 0x39;
constexpr uint8_t REGISTER_NVM_PAR_P3 = 0x3A;
constexpr uint8_t REGISTER_NVM_PAR_P4 = 0x3B;
constexpr uint8_t REGISTER_NVM_PAR_P5_LSB = 0x3C;
constexpr uint8_t REGISTER_NVM_PAR_P5_MSB = 0x3D;
constexpr uint8_t REGISTER_NVM_PAR_P6_LSB = 0x3E;
constexpr uint8_t REGISTER_NVM_PAR_P6_MSB = 0x3F;
constexpr uint8_t REGISTER_NVM_PAR_P7 = 0x40;
constexpr uint8_t REGISTER_NVM_PAR_P8 = 0x41;
constexpr uint8_t REGISTER_NVM_PAR_P9_LSB = 0x42;
constexpr uint8_t REGISTER_NVM_PAR_P9_MSB = 0x43;
constexpr uint8_t REGISTER_NVM_PAR_P10 = 0x44;
constexpr uint8_t REGISTER_NVM_PAR_P11 = 0x45;
// 0x46 - 0x7D reserved
constexpr uint8_t REGISTER_CMD = 0x7E;

// Alias
constexpr uint8_t REGISTER_PRESS_DATA = REGISTER_DATA_0;
constexpr uint8_t REGISTER_TEMP_DATA = REGISTER_DATA_3;

//////////////////////////////////////////
// Register unions
//////////////////////////////////////////
typedef union { // Register 0x00 "CHIP_ID"
    struct {
        uint8_t chip_id_nvm : 4;
        uint8_t chip_id_fixed : 4;
    } bit;
    uint8_t reg = 0;
} RegisterChipId;

typedef union { // Register 0x01 "REV_ID"
    struct {
        uint8_t rev_id_minor : 4;
        uint8_t rev_id_major : 4;
    } bit;
    uint8_t reg = 0;
} RegisterRevId;

typedef union { // Register 0x02 "ERR_REG"
    struct {
        uint8_t fatal_err : 1; // Fatal error
        uint8_t cmd_err : 1; // Command execution failed. Cleared on read.
        uint8_t conf_err : 1; // Sensor configuration error detected (only working in normal mode). Cleared on read.
    } bit;
    uint8_t reg = 0;
} RegisterErrReg;

typedef union { // Register 0x03 "STATUS"
    struct {
        uint8_t _ : 4; // Reserved
        uint8_t cmd_rdy : 1; // CMD decoder status. (0 = Command in progress; 1 = Command decoder is ready to accept a new command)
        uint8_t drdy_press : 1; // Data ready for pressure. It gets reset, when one pressure DATA register is read out.
        uint8_t drdy_temp : 1; // Data ready for temperature sensor. It gets reset, when one temperature DATA register is read out.
    } bit;
    uint8_t reg = 0;
} RegisterStatus;

// Register 0x04-0x06 "Pressure Data"
// Register 0x07-0x09 "Temperature Data"
// Register 0x0C-0x0E "Sensor Time"

typedef union { // Register 0x10 "EVENT"
    struct {
        uint8_t por_detected : 1; // (1 = after device power up or softreset. Clear-on-read)
        uint8_t itf_act_pt : 1; // (1 = when a serial interface transaction occurs during a pressure or temperature conversion. Clear-on-read)
    } bit;
    uint8_t reg = 0;
} RegisterEvent;

typedef union { // Register 0x11 "INT_STATUS"
    struct {
        uint8_t fwm_int : 1; // FIFO Watermark Interrupt
        uint8_t ffull_int : 1; // FIFO Full Interrupt
        uint8_t drdy : 1; // Data ready interrupt
    } bit;
    uint8_t reg = 0;
} RegisterIntStatus;

// Register 0x12-0x13 "FIFO_LENGTH"
// Register 0x14 "FIFO_DATA"
// Register 0x15-0x16 "FIFO Watermark"

typedef union { // Register 0x17 "FIFO_CONFIG_1"
    struct {
        uint8_t fifo_mode : 1; // Enables or disables the FIFO (0 = disable; 1 = enable)
        uint8_t fifo_stop_on_full : 1; // Stop writing samples into FIFO when FIFO is full. (0 = do not stop writing to FIFO when full; 1 = stop writing into FIFO when full.)
        uint8_t fifo_time_en : 1; // Return sensortime frame after the last valid data frame. (0 = do not return sensortime frame; 1 = return sensortime frame)
        uint8_t fifo_press_en : 1; // Store pressure data in FIFO (0 = no pressure data is stored; 1 = pressure data is stored)
        uint8_t fifo_temp_en : 1; // Store temperature data in FIFO (0 = no temperature data is stored; 1 = temperature data is stored)
    } bit;
    uint8_t reg = 0;
} RegisterFifoConfig1;

typedef union { // Register 0x18 "FIFO_CONFIG_2"
    struct {
        uint8_t fifo_subsampling : 3; // FIFO downsampling selection for pressure and temperature data, factor is 2^fifo_subsampling
        uint8_t data_select : 2; // For pressure and temperature, select data source. (00 = unfiltered data [compensated or uncompensated]; 01 = filtered data [compensated or uncompensated])
    } bit;
    uint8_t reg = 0;
} RegisterFifoConfig2;

enum FifoConfig2FifoSubsampling {
    Subsampling_1 = 0x00,
    Subsampling_2 = 0x01,
    Subsampling_4 = 0x02,
    Subsampling_8 = 0x03,
    Subsampling_16 = 0x04,
    Subsampling_32 = 0x05,
    Subsampling_64 = 0x06,
    Subsampling_128 = 0x07
};

enum FifoConfig2DataSelect {
    UNFILTERED = 0x0,
    FILTERED = 0x1
};

// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_1   0x00
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_2   0x01
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_4   0x02
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_8   0x03
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_16  0x04
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_32  0x05
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_64  0x06
// #define REGISTER_FIFO_CONFIG_2_FIFO_SUBSAMPLING_128 0x07

// #define REGISTER_FIFO_CONFIG_2_DATA_SELECT_UNFILTERED   0x0
// #define REGISTER_FIFO_CONFIG_2_DATA_SELECT_FILTERED     0x1

typedef union { // Register 0x19 "INT_CTRL"
    struct {
        uint8_t int_od : 1; // Configure output: open-drain or push-pull (0 = push-pull; 1 = open-drain)
        uint8_t int_level : 1; // Level of INT pin (0 = active_low; 1 = active_high)
        uint8_t int_latch : 1; // Latching of interrupts for INT pin and INT_STATUS register (0 = disabled; 1 = enabled)
        uint8_t fwtm_en : 1; // enable FIFO watermark reached interrupt for INT pin and INT_STATUS. (0 = disabled; 1 = enabled)
        uint8_t ffull_en : 1; // enable Fifo full interrupt for INT pin and INT_STATUS (0 = disabled; 1 = enabled)
        uint8_t int_ds : 1; // (0 = low; 1 = high)
        uint8_t drdy_en : 1; // Enable temperature / pressure data ready interrupt for INT pin and INT_STATUS (0 = disabled; 1 = enabled)
    } bit;
    uint8_t reg = 0;
} RegisterIntCtrl;

typedef union { // Register 0x1A "IF_CONF"
    struct {
        uint8_t spi3 : 1; // Configure SPI Interface Mode for primary interface (0 = SPI 4-wire mode; 1 = SPI 3-wire mode)
        uint8_t i2c_wdt_en : 1; // Enable for the I2C Watchdog timer, backed by NVM (0 = Watchdog disabled; 1 = Watchdog enabled)
        uint8_t i2c_wdt_sel : 1; // Select timer period for I2C Watchdog , backed by NVM (0 = I2C watchdog timeout after 1.25 ms; 1 = I2C watchdog timeout after 40 ms)
    } bit;
    uint8_t reg = 0;
} RegisterIfConf;

typedef union { // Register 0x1B "PWR_CTRL"
    struct {
        uint8_t press_en : 1; // (0 = Disables the pressure sensor; 1 = Enables the pressure sensor)
        uint8_t temp_en : 1; // (0 = Disables the temperature sensor; 1 = Enables the temperature sensor)
        uint8_t _ : 2; // Reserved
        uint8_t mode : 2; // (00 = sleep mode; 01 or 10 = forced mode; 11 = normal mode)
    } bit;
    uint8_t reg = 0;
} RegisterPwrCtrl;

enum PwrCtrlMode {
    sleep = 0x0,
    forced = 0x1,
    normal = 0x3,
};

// #define REGISTER_PWR_CTRL_MODE_SLEEP    0x0
// #define REGISTER_PWR_CTRL_MODE_FORCED   0x1
// #define REGISTER_PWR_CTRL_MODE_NORMAL   0x3

typedef union { // Register 0x1C "OSR"
    struct {
        uint8_t osr_p : 3; // Oversampling setting pressure measurement
        uint8_t osr_t : 3; // Oversampling setting temperature measurement
    } bit;
    uint8_t reg = 0;
} RegisterOsr;

enum OsrOsr {
    NONE = 0x0,
    X2 = 0x1,
    X4 = 0x2,
    X8 = 0x3,
    X16 = 0x4,
    X32 = 0x5
};

// #define REGISTER_OSR_OSR_X_NONE 0x0
// #define REGISTER_OSR_OSR_X_X2   0x1
// #define REGISTER_OSR_OSR_X_X4   0x2
// #define REGISTER_OSR_OSR_X_X8   0x3
// #define REGISTER_OSR_OSR_X_X16  0x4
// #define REGISTER_OSR_OSR_X_X32  0x5

typedef union { // Register 0x1D "ODR"
    struct {
        uint8_t odr_sel : 5; // Subdivision factor for pressure and temperature measurements is 2^value. Allowed values are 0..17. Other values are saturated at 17.
    } bit;
    uint8_t reg = 0;
} RegisterOdr;

enum OdrOdrSel {
    Prescaler1 = 0x00, // ODR 200Hz - 5ms
    Prescaler2 = 0x01, // ODR 100Hz - 10ms
    Prescaler4 = 0x02, // ODR 50Hz - 20ms
    Prescaler8 = 0x03, // ODR 25Hz - 40ms
    Prescaler16 = 0x04, // ODR 25/2Hz - 80ms
    Prescaler32 = 0x05, // ODR 25/4Hz - 160ms
    Prescaler64 = 0x06, // ODR 25/8Hz - 320ms
    Prescaler128 = 0x07, // ODR 25/16Hz - 640ms
    Prescaler256 = 0x08, // ODR 25/32Hz - 1.280s
    Prescaler512 = 0x09, // ODR 25/64Hz - 2.560s
    Prescaler1024 = 0x0A, // ODR 25/128Hz - 5.120s
    Prescaler2048 = 0x0B, // ODR 25/256Hz - 10.24s
    Prescaler4096 = 0x0C, // ODR 25/512Hz - 20.48s
    Prescaler8192 = 0x0D, // ODR 25/1024Hz - 40.96s
    Prescaler16384 = 0x0E, // ODR 25/2048Hz - 81.92s
    Prescaler32768 = 0x0F, // ODR 25/4096Hz - 163.84s
    Prescaler65536 = 0x10, // ODR 25/8192Hz - 327.68s
    Prescaler131072 = 0x11 // ODR 25/16384Hz - 655.36s
};

// #define REGISTER_ODR_ODR_SEL_PRESCALER_1        0x00 // ODR 200Hz - 5ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_2        0x01 // ODR 100Hz - 10ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_4        0x02 // ODR 50Hz - 20ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_8        0x03 // ODR 25Hz - 40ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_16       0x04 // ODR 25/2Hz - 80ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_32       0x05 // ODR 25/4Hz - 160ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_64       0x06 // ODR 25/8Hz - 320ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_128      0x07 // ODR 25/16Hz - 640ms
// #define REGISTER_ODR_ODR_SEL_PRESCALER_256      0x08 // ODR 25/32Hz - 1.280s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_512      0x09 // ODR 25/64Hz - 2.560s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_1024     0x0A // ODR 25/128Hz - 5.120s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_2048     0x0B // ODR 25/256Hz - 10.24s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_4096     0x0C // ODR 25/512Hz - 20.48s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_8192     0x0D // ODR 25/1024Hz - 40.96s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_16384    0x0E // ODR 25/2048Hz - 81.92s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_32768    0x0F // ODR 25/4096Hz - 163.84s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_65536    0x10 // ODR 25/8192Hz - 327.68s
// #define REGISTER_ODR_ODR_SEL_PRESCALER_131072   0x11 // ODR 25/16384Hz - 655.36s

typedef union { // Register 0x1F "CONFIG"
    struct {
        uint8_t _ : 1; // Reserved
        uint8_t iir_filter : 3; // Filter coefficient for IIR filter
    } bit;
    uint8_t reg = 0;
} RegisterConfig;

enum ConfigIRRFilter {
    Coefficient_0 = 0x0,
    Coefficient_1 = 0x1,
    Coefficient_3 = 0x2,
    Coefficient_7 = 0x3,
    Coefficient_15 = 0x4,
    Coefficient_31 = 0x5,
    Coefficient_63 = 0x6,
    Coefficient_127 = 0x7
};

// #define REGISTER_CONFIG_IRR_FILTER_COEF_0   0x0
// #define REGISTER_CONFIG_IRR_FILTER_COEF_1   0x1
// #define REGISTER_CONFIG_IRR_FILTER_COEF_3   0x2
// #define REGISTER_CONFIG_IRR_FILTER_COEF_7   0x3
// #define REGISTER_CONFIG_IRR_FILTER_COEF_15  0x4
// #define REGISTER_CONFIG_IRR_FILTER_COEF_31  0x5
// #define REGISTER_CONFIG_IRR_FILTER_COEF_63  0x6
// #define REGISTER_CONFIG_IRR_FILTER_COEF_127 0x7

typedef union { // Register 0x7E "CMD"
    struct {
        uint8_t _ : 1; // Reserved
        uint8_t fifo_flush : 1; // Clears all data in the FIFO, does not change FIFO_CONFIG registers
        uint8_t softreset : 1; // Triggers a reset, all user configuration settings are overwritten with their default state
    } bit;
    uint8_t reg = 0;
} RegisterCMD;

struct MemoryMapTrimmingCoefficients {
    uint16_t NVM_PAR_T1;
    uint16_t NVM_PAR_T2;
    int8_t NVM_PAR_T3;

    int16_t NVM_PAR_P1;
    int16_t NVM_PAR_P2;
    int8_t NVM_PAR_P3;
    int8_t NVM_PAR_P4;
    uint16_t NVM_PAR_P5;
    uint16_t NVM_PAR_P6;
    int8_t NVM_PAR_P7;
    int8_t NVM_PAR_P8;
    int16_t NVM_PAR_P9;
    int8_t NVM_PAR_P10;
    int8_t NVM_PAR_P11;
};

struct CalibrationCoefficents {
    float PAR_T1;
    float PAR_T2;
    float PAR_T3;

    float PAR_P1;
    float PAR_P2;
    float PAR_P3;
    float PAR_P4;
    float PAR_P5;
    float PAR_P6;
    float PAR_P7;
    float PAR_P8;
    float PAR_P9;
    float PAR_P10;
    float PAR_P11;
};

class BMP390 : SPIDevice {
private:
    CalibrationCoefficents calibrationCoefficents;
    float calibrationTemperature;

public:
    BMP390(SPIClass* spiBus, SPISettings settings, uint8_t slaveSelectPin);
    virtual ~BMP390() = default;

    void begin();

    // CHIP_ID
    uint8_t getChipId();

    // REV_ID
    uint8_t getRevId();

    // ERR_REG
    RegisterErrReg getErrors();

    // STATUS
    bool isCMDDecoderAvailable();
    bool isPressureDataAvailable();
    bool isTemperatureDataAvailable();

    // DATA
    uint32_t readPressure();
    float readCompensatedPressure();
    uint32_t readTemperature();
    float readCompensatedTemperature();
    uint32_t readSensorTime();

    // EVENT
    RegisterEvent getEvents();

    // INT_STATUS
    RegisterIntStatus getInterruptStatuses();

    // FIFO_LENGTH
    uint16_t getFIFOLength();

    // FIFO_DATA
    uint8_t readFIFO();

    // FIFO Watermark
    void setFIFOWatermark(uint16_t level);
    uint16_t getFIFOWatermark();

    // FIFO_CONFIG_1
    void setFIFOMode(bool enabled);
    bool getFIFOMode();
    void setFIFOStopOnFull(bool enabled);
    bool getFIFOStopOnFull();
    void setFIFOSensorTimeFrame(bool enabled);
    bool getFIFOSensorTimeFrame();
    void setFIFOStorePressure(bool enabled);
    bool getFIFOStorePressure();
    void setFIFOStoreTemperature(bool enabled);
    bool getFIFOStoreTemperature();

    // FIFO_CONFIG_2
    void setFIFOSubsampling(FifoConfig2FifoSubsampling sample);
    FifoConfig2FifoSubsampling getFIFOSubsampling();
    void setFIFODataFilter(FifoConfig2DataSelect selection);
    FifoConfig2DataSelect setFIFODataFilter();

    // INT_CTRL
    void setInterruptPinMode(bool isOpenDrain);
    bool getInterruptPinMode();
    void setInterruptLevel(bool isActiveHigh);
    bool getInterruptLevel();
    void setInterruptLatching(bool enabled);
    bool getInterruptLatching();
    void setFIFOWatermarkInterruptStatus(bool enabled);
    bool getFIFOWatermarkInterruptStatus();
    void setFIFOFullInterruptStatus(bool enabled);
    bool getFIFOFullInterruptStatus();
    void setInterruptDS(bool isHigh); // TODO: What is DS?
    bool getInterruptDS(); // TODO: What is DS?
    void setDataReadyInterruptStatus(bool enabled);
    bool getDataReadyInterruptStatus();

    // IF_CONF
    void setSPIMode(bool is3Wire);
    bool getSPIMode();

    // PWR_CTRL
    void setPowerMode(PwrCtrlMode mode);
    PwrCtrlMode getPowerMode();
    void setPressureSensorStatus(bool enabled);
    bool getPressureSensorStatus();
    void setTemperatureSensorStatus(bool enabled);
    bool getTemperatureSensorStatus();

    // OSR
    void setPressureOversampling(OsrOsr osr);
    OsrOsr getPressureOversampling();
    void setTemperatureOversampling(OsrOsr osr);
    OsrOsr getTemperatureOversampling();

    // ODR
    void setOutputDataRate(OdrOdrSel odrSel);
    OdrOdrSel getOutputDataRate();

    // CONFIG
    void setIIRFilter(ConfigIRRFilter filter);
    ConfigIRRFilter getIIRFilter();

    // CMD
    void flushFIFO();
    void softReset();

    // TRIM
    void readCalibrationCoefficents();
};

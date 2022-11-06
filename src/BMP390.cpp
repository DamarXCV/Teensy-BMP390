#include "BMP390.h"

BMP390::BMP390(SPIClass* spiBus, SPISettings settings, uint8_t slaveSelectPin)
    : SPIDevice(spiBus, settings, slaveSelectPin)
{
}

void BMP390::begin()
{
    softReset();
    delay(100); // TODO: get ready time

    readCalibrationCoefficents();

    bool currentTempStatus = getTemperatureSensorStatus();
    setTemperatureSensorStatus(1);
    readCompensatedTemperature();
    if (currentTempStatus == 0) {
        setTemperatureSensorStatus(0);
    }
}

uint8_t BMP390::getChipId()
{
    RegisterChipId chipIdReg;

    chipIdReg.reg = read8(REGISTER_CHIP_ID);

    return chipIdReg.reg;
}

uint8_t BMP390::getRevId()
{
    RegisterRevId revIdReg;

    revIdReg.reg = read8(REGISTER_REV_ID);

    return revIdReg.reg;
}

RegisterErrReg BMP390::getErrors()
{
    RegisterErrReg errRegReg;

    errRegReg.reg = read8(REGISTER_ERR_REG);

    return errRegReg;
}

bool BMP390::isCMDDecoderAvailable()
{
    RegisterStatus statusReg;

    statusReg.reg = read8(REGISTER_STATUS);

    return statusReg.bit.cmd_rdy;
}

bool BMP390::isPressureDataAvailable()
{
    RegisterStatus statusReg;

    statusReg.reg = read8(REGISTER_STATUS);

    return statusReg.bit.drdy_press;
}

bool BMP390::isTemperatureDataAvailable()
{
    RegisterStatus statusReg;

    statusReg.reg = read8(REGISTER_STATUS);

    return statusReg.bit.drdy_temp;
}

uint32_t BMP390::readPressure()
{
    uint8_t data[3];

    readBuffer(REGISTER_DATA_0, data, 3);

    return (((uint32_t)data[0]) << 0) | (((uint32_t)data[1]) << 8) | (((uint32_t)data[2]) << 16);
}

float BMP390::readCompensatedPressure()
{
    uint32_t pressure = readPressure();

    // Temporary variables used for compensation
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    // Calibration data
    partial_data1 = calibrationCoefficents.PAR_P6 * calibrationTemperature;
    partial_data2 = calibrationCoefficents.PAR_P7 * (calibrationTemperature * calibrationTemperature);
    partial_data3 = calibrationCoefficents.PAR_P8 * (calibrationTemperature * calibrationTemperature * calibrationTemperature);
    partial_out1 = calibrationCoefficents.PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calibrationCoefficents.PAR_P2 * calibrationTemperature;
    partial_data2 = calibrationCoefficents.PAR_P3 * (calibrationTemperature * calibrationTemperature);
    partial_data3 = calibrationCoefficents.PAR_P4 * (calibrationTemperature * calibrationTemperature * calibrationTemperature);
    partial_out2 = (float)pressure * (calibrationCoefficents.PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)pressure * (float)pressure;
    partial_data2 = calibrationCoefficents.PAR_P9 + calibrationCoefficents.PAR_P10 * calibrationTemperature;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)pressure * (float)pressure * (float)pressure) * calibrationCoefficents.PAR_P11;

    return partial_out1 + partial_out2 + partial_data4;
}

uint32_t BMP390::readTemperature()
{
    uint8_t data[3];

    readBuffer(REGISTER_DATA_3, data, 3);

    return (((uint32_t)data[0]) << 0) | (((uint32_t)data[1]) << 8) | (((uint32_t)data[2]) << 16);
}

float BMP390::readCompensatedTemperature()
{
    uint32_t temperature = readTemperature();

    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(temperature - calibrationCoefficents.PAR_T1);
    partial_data2 = (float)(partial_data1 * calibrationCoefficents.PAR_T2);

    // Update the compensated temperature in calib structure since this is needed for pressure calculation
    calibrationTemperature = partial_data2 + (partial_data1 * partial_data1) * calibrationCoefficents.PAR_T3;

    return calibrationTemperature;
}

uint32_t BMP390::readSensorTime()
{
    uint8_t data[3];

    readBuffer(REGISTER_SENSORTIME_0, data, 3);

    return (((uint32_t)data[0]) << 0) | (((uint32_t)data[1]) << 8) | (((uint32_t)data[2]) << 16);
}

RegisterEvent BMP390::getEvents()
{
    RegisterEvent eventReg;

    eventReg.reg = read8(REGISTER_EVENT);

    return eventReg;
}

RegisterIntStatus BMP390::getInterruptStatuses()
{
    RegisterIntStatus intStatusReg;

    intStatusReg.reg = read8(REGISTER_INT_STATUS);

    return intStatusReg;
}

uint16_t BMP390::getFIFOLength()
{
    uint8_t data[2];

    readBuffer(REGISTER_FIFO_LENGTH_0, data, 2);

    return ((uint16_t)data[0]) << 8 | (data[1] & 0x1);
}

uint8_t BMP390::readFIFO()
{
    return read8(REGISTER_FIFO_DATA);
}

void BMP390::setFIFOWatermark(uint16_t level)
{
    uint16_t newLevel = (level > 511) ? 511 : level;

    // TODO: Write with one command?
    write8(REGISTER_FIFO_WTM_0, (uint8_t)newLevel);
    write8(REGISTER_FIFO_WTM_1, (uint8_t)(newLevel >> 8));
}

uint16_t BMP390::getFIFOWatermark()
{
    uint8_t data[2];

    readBuffer(REGISTER_FIFO_WTM_0, data, 2);

    return ((uint16_t)data[0]) << 8 | (data[1] & 0x1);
}

void BMP390::setFIFOMode(bool enabled)
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    // Set mode
    fifoConfig1Reg.bit.fifo_mode = enabled;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_1, fifoConfig1Reg.reg);
}

bool BMP390::getFIFOMode()
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    return fifoConfig1Reg.bit.fifo_mode;
}

void BMP390::setFIFOStopOnFull(bool enabled)
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    // Set stop on full
    fifoConfig1Reg.bit.fifo_stop_on_full = enabled;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_1, fifoConfig1Reg.reg);
}

bool BMP390::getFIFOStopOnFull()
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    return fifoConfig1Reg.bit.fifo_stop_on_full;
}

void BMP390::setFIFOSensorTimeFrame(bool enabled)
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    // Set sensortime frame
    fifoConfig1Reg.bit.fifo_time_en = enabled;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_1, fifoConfig1Reg.reg);
}

bool BMP390::getFIFOSensorTimeFrame()
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    return fifoConfig1Reg.bit.fifo_time_en;
}

void BMP390::setFIFOStorePressure(bool enabled)
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    // Set pressure collection
    fifoConfig1Reg.bit.fifo_press_en = enabled;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_1, fifoConfig1Reg.reg);
}

bool BMP390::getFIFOStorePressure()
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    return fifoConfig1Reg.bit.fifo_press_en;
}

void BMP390::setFIFOStoreTemperature(bool enabled)
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    // Set temperature collection
    fifoConfig1Reg.bit.fifo_temp_en = enabled;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_1, fifoConfig1Reg.reg);
}

bool BMP390::getFIFOStoreTemperature()
{
    RegisterFifoConfig1 fifoConfig1Reg;

    // Get register values
    fifoConfig1Reg.reg = read8(REGISTER_FIFO_CONFIG_1);

    return fifoConfig1Reg.bit.fifo_temp_en;
}

void BMP390::setFIFOSubsampling(FifoConfig2FifoSubsampling sample)
{
    RegisterFifoConfig2 fifoConfig2Reg;

    // Get register values
    fifoConfig2Reg.reg = read8(REGISTER_FIFO_CONFIG_2);

    // Set subsampling
    fifoConfig2Reg.bit.fifo_subsampling = sample;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_2, fifoConfig2Reg.reg);
}

FifoConfig2FifoSubsampling BMP390::getFIFOSubsampling()
{
    RegisterFifoConfig2 fifoConfig2Reg;

    // Get register values
    fifoConfig2Reg.reg = read8(REGISTER_FIFO_CONFIG_2);

    return (FifoConfig2FifoSubsampling)fifoConfig2Reg.bit.fifo_subsampling;
}

void BMP390::setFIFODataFilter(FifoConfig2DataSelect selection)
{
    RegisterFifoConfig2 fifoConfig2Reg;

    // Get register values
    fifoConfig2Reg.reg = read8(REGISTER_FIFO_CONFIG_2);

    // Set unfiltered or filter
    fifoConfig2Reg.bit.data_select = selection;

    // Write register values
    write8(REGISTER_FIFO_CONFIG_2, fifoConfig2Reg.reg);
}

FifoConfig2DataSelect BMP390::setFIFODataFilter()
{
    RegisterFifoConfig2 fifoConfig2Reg;

    // Get register values
    fifoConfig2Reg.reg = read8(REGISTER_FIFO_CONFIG_2);

    return (FifoConfig2DataSelect)fifoConfig2Reg.bit.data_select;
}

void BMP390::setInterruptPinMode(bool isOpenDrain)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set push-pull or open-drain
    intCtrlReg.bit.int_od = isOpenDrain;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getInterruptPinMode()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.int_od;
}

void BMP390::setInterruptLevel(bool isActiveHigh)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set active low or active high
    intCtrlReg.bit.int_level = isActiveHigh;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getInterruptLevel()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.int_level;
}

void BMP390::setInterruptLatching(bool enabled)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set disabled or enabled
    intCtrlReg.bit.int_latch = enabled;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getInterruptLatching()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.int_latch;
}

void BMP390::setFIFOWatermarkInterruptStatus(bool enabled)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set disabled or enabled
    intCtrlReg.bit.fwtm_en = enabled;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getFIFOWatermarkInterruptStatus()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.fwtm_en;
}

void BMP390::setFIFOFullInterruptStatus(bool enabled)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set disabled or enabled
    intCtrlReg.bit.ffull_en = enabled;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getFIFOFullInterruptStatus()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.ffull_en;
}

void BMP390::setInterruptDS(bool isHigh)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set low or high
    intCtrlReg.bit.int_ds = isHigh;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getInterruptDS()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.int_ds;
}

void BMP390::setDataReadyInterruptStatus(bool enabled)
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    // Set disabled or enabled
    intCtrlReg.bit.drdy_en = enabled;

    // Write register values
    write8(REGISTER_INT_CTRL, intCtrlReg.reg);
}

bool BMP390::getDataReadyInterruptStatus()
{
    RegisterIntCtrl intCtrlReg;

    // Get register values
    intCtrlReg.reg = read8(REGISTER_INT_CTRL);

    return intCtrlReg.bit.drdy_en;
}

void BMP390::setSPIMode(bool is3Wire)
{
    RegisterIfConf ifConfReg;

    // Get register values
    ifConfReg.reg = read8(REGISTER_IF_CONF);

    // Set mode
    ifConfReg.bit.spi3 = is3Wire;

    // Write register values
    write8(REGISTER_IF_CONF, ifConfReg.reg);
}

bool BMP390::getSPIMode()
{
    RegisterIfConf ifConfReg;

    // Get register values
    ifConfReg.reg = read8(REGISTER_IF_CONF);

    return ifConfReg.bit.spi3;
}

void BMP390::setPowerMode(PwrCtrlMode mode)
{
    RegisterPwrCtrl pwrCtrlReg;

    // Get register values
    pwrCtrlReg.reg = read8(REGISTER_PWR_CTRL);

    // Set mode to Sleep if not in Sleep
    if (pwrCtrlReg.bit.mode != PwrCtrlMode::sleep) {
        pwrCtrlReg.bit.mode = PwrCtrlMode::sleep;
        write8(REGISTER_PWR_CTRL, pwrCtrlReg.reg);
        delayMicroseconds(100);
    }

    // Enter the desired mode
    pwrCtrlReg.bit.mode = mode;

    // Write register values
    write8(REGISTER_PWR_CTRL, pwrCtrlReg.reg);
}

PwrCtrlMode BMP390::getPowerMode()
{
    RegisterPwrCtrl pwrCtrlReg;

    // Get current mode
    pwrCtrlReg.reg = read8(REGISTER_PWR_CTRL);

    return (PwrCtrlMode)pwrCtrlReg.bit.mode;
}

void BMP390::setPressureSensorStatus(bool enabled)
{
    RegisterPwrCtrl pwrCtrlReg;

    // Get register values
    pwrCtrlReg.reg = read8(REGISTER_PWR_CTRL);

    // Set pressure sensor status
    pwrCtrlReg.bit.press_en = enabled;

    // Write register values
    write8(REGISTER_PWR_CTRL, pwrCtrlReg.reg);
}

bool BMP390::getPressureSensorStatus()
{
    RegisterPwrCtrl pwrCtrlReg;

    // Get register values
    pwrCtrlReg.reg = read8(REGISTER_PWR_CTRL);

    return pwrCtrlReg.bit.press_en;
}

void BMP390::setTemperatureSensorStatus(bool enabled)
{
    RegisterPwrCtrl pwrCtrlReg;

    // Get register values
    pwrCtrlReg.reg = read8(REGISTER_PWR_CTRL);

    // Set pressure sensor status
    pwrCtrlReg.bit.temp_en = enabled;

    // Write register values
    write8(REGISTER_PWR_CTRL, pwrCtrlReg.reg);
}

bool BMP390::getTemperatureSensorStatus()
{
    RegisterPwrCtrl pwrCtrlReg;

    // Get register values
    pwrCtrlReg.reg = read8(REGISTER_PWR_CTRL);

    return pwrCtrlReg.bit.temp_en;
}

void BMP390::setPressureOversampling(OsrOsr osr)
{
    RegisterOsr osrReg;

    // Get register values
    osrReg.reg = read8(REGISTER_OSR);

    // Set oversamplingrate
    osrReg.bit.osr_p = osr;

    // Write register values
    write8(REGISTER_OSR, osrReg.reg);
}

OsrOsr BMP390::getPressureOversampling()
{
    RegisterOsr osrReg;

    // Get register values
    osrReg.reg = read8(REGISTER_OSR);

    return (OsrOsr)osrReg.bit.osr_p;
}

void BMP390::setTemperatureOversampling(OsrOsr osr)
{
    RegisterOsr osrReg;

    // Get register values
    osrReg.reg = read8(REGISTER_OSR);

    // Set oversamplingrate
    osrReg.bit.osr_t = osr;

    // Write register values
    write8(REGISTER_OSR, osrReg.reg);
}

OsrOsr BMP390::getTemperatureOversampling()
{
    RegisterOsr osrReg;

    // Get register values
    osrReg.reg = read8(REGISTER_OSR);

    return (OsrOsr)osrReg.bit.osr_t;
}

void BMP390::setOutputDataRate(OdrOdrSel odrSel)
{
    RegisterOdr odrReg;

    // Get register values
    odrReg.reg = read8(REGISTER_ODR);

    // Set oversamplingrate
    odrReg.bit.odr_sel = odrSel;

    // Write register values
    write8(REGISTER_ODR, odrReg.reg);
}

OdrOdrSel BMP390::getOutputDataRate()
{
    RegisterOdr odrReg;

    // Get register values
    odrReg.reg = read8(REGISTER_ODR);

    return (OdrOdrSel)odrReg.bit.odr_sel;
}

void BMP390::setIIRFilter(ConfigIRRFilter filter)
{
    RegisterConfig configReg;

    // Get register values
    configReg.reg = read8(REGISTER_CONFIG);

    // Set filter
    configReg.bit.iir_filter = filter;

    // Write register values
    write8(REGISTER_CONFIG, configReg.reg);
}

ConfigIRRFilter BMP390::getIIRFilter()
{
    RegisterConfig configReg;

    // Get register values
    configReg.reg = read8(REGISTER_CONFIG);

    return (ConfigIRRFilter)configReg.bit.iir_filter;
}

void BMP390::flushFIFO()
{
    RegisterCMD cmdReg;
    cmdReg.bit.fifo_flush = 1;

    write8(REGISTER_CMD, cmdReg.reg);
}

void BMP390::softReset()
{
    RegisterCMD cmdReg;
    cmdReg.bit.softreset = 1;

    write8(REGISTER_CMD, cmdReg.reg);
}

void BMP390::readCalibrationCoefficents()
{
    uint8_t values[21];

    readBuffer(REGISTER_NVM_PAR_T1_LSB, values, 21);

    MemoryMapTrimmingCoefficients trimmingCoefficients;

    trimmingCoefficients.NVM_PAR_T1 = ((uint16_t)values[0]) << 0 | ((uint16_t)values[1]) << 8;
    trimmingCoefficients.NVM_PAR_T2 = ((uint16_t)values[2]) << 0 | ((uint16_t)values[3]) << 8;
    trimmingCoefficients.NVM_PAR_T3 = values[4];

    trimmingCoefficients.NVM_PAR_P1 = ((uint16_t)values[5]) << 0 | ((uint16_t)values[6]) << 8;
    trimmingCoefficients.NVM_PAR_P2 = ((uint16_t)values[7]) << 0 | ((uint16_t)values[8]) << 8;
    trimmingCoefficients.NVM_PAR_P3 = values[9];
    trimmingCoefficients.NVM_PAR_P4 = values[10];
    trimmingCoefficients.NVM_PAR_P5 = ((uint16_t)values[11]) << 0 | ((uint16_t)values[12]) << 8;
    trimmingCoefficients.NVM_PAR_P6 = ((uint16_t)values[13]) << 0 | ((uint16_t)values[14]) << 8;
    trimmingCoefficients.NVM_PAR_P7 = values[15];
    trimmingCoefficients.NVM_PAR_P8 = values[16];
    trimmingCoefficients.NVM_PAR_P9 = ((uint16_t)values[17]) << 0 | ((uint16_t)values[18]) << 8;
    trimmingCoefficients.NVM_PAR_P10 = values[19];
    trimmingCoefficients.NVM_PAR_P11 = values[20];

    calibrationCoefficents.PAR_T1 = trimmingCoefficients.NVM_PAR_T1 / pow(2.0f, -8.0f);
    calibrationCoefficents.PAR_T2 = trimmingCoefficients.NVM_PAR_T2 / pow(2.0f, 30.0f);
    calibrationCoefficents.PAR_T3 = trimmingCoefficients.NVM_PAR_T3 / pow(2.0f, 48.0f);

    calibrationCoefficents.PAR_P1 = (trimmingCoefficients.NVM_PAR_P1 - pow(2.0f, 14)) / pow(2.0f, 20.0f);
    calibrationCoefficents.PAR_P2 = (trimmingCoefficients.NVM_PAR_P2 - pow(2.0f, 14)) / pow(2.0f, 29.0f);
    calibrationCoefficents.PAR_P3 = trimmingCoefficients.NVM_PAR_P3 / pow(2.0f, 32.0f);
    calibrationCoefficents.PAR_P4 = trimmingCoefficients.NVM_PAR_P4 / pow(2.0f, 37.0f);
    calibrationCoefficents.PAR_P5 = trimmingCoefficients.NVM_PAR_P5 / pow(2.0f, -3.0f);
    calibrationCoefficents.PAR_P6 = trimmingCoefficients.NVM_PAR_P6 / pow(2.0f, 6.0f);
    calibrationCoefficents.PAR_P7 = trimmingCoefficients.NVM_PAR_P7 / pow(2.0f, 8.0f);
    calibrationCoefficents.PAR_P8 = trimmingCoefficients.NVM_PAR_P8 / pow(2.0f, 15.0f);
    calibrationCoefficents.PAR_P9 = trimmingCoefficients.NVM_PAR_P9 / pow(2.0f, 48.0f);
    calibrationCoefficents.PAR_P10 = trimmingCoefficients.NVM_PAR_P10 / pow(2.0f, 48.0f);
    calibrationCoefficents.PAR_P11 = trimmingCoefficients.NVM_PAR_P11 / pow(2.0f, 65.0f);
}

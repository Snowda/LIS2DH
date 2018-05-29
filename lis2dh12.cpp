/** Based on ST MicroElectronics LIS2DH datasheet http://www.st.com/web/en/resource/technical/document/datasheet/DM00042751.pdf
* 30/09/2014 by Conor Forde <me@conorforde.com>
* Updates should be available at https://github.com/Snowda/LIS2DH
*
* Changelog:
*     ... - ongoing development release

* NOTE: THIS IS ONLY A PARIAL RELEASE. 
* THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE DEVELOPMENT AND IS MISSING MOST FEATURES. 
* PLEASE KEEP THIS IN MIND IF YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
*/
#include "Arduino.h"
#include "stdint.h"
#include "lis2dh12.h"
#include "Wire.h"

/**
 * Set the LIS2DH chip address based on the configuration selected for
 * SA0 pin.
 */
LIS2DH::LIS2DH( uint8_t sa0 ) {
  switch ( sa0 ) {
    case LOW :
        _address = LIS2DH_LIS2DH_SA0_LOW;    
        break;
    case HIGH :
        _address = LIS2DH_LIS2DH_SA0_HIGH;
        break;    
    default:
        _address = LIS2DH_LIS2DH_SA0_LOW;    
  }
}


bool LIS2DH::init(void) {
    Wire.begin(); 
    
    //Put into the correct operating mode 
    disableLowPower();
    enableAxisXYZ();
    setDataRate(2);
}

// -----------------------------------------------------
// Write to LIS2DH
// -----------------------------------------------------

/**
 * Write a 8b register on the chip
 */
bool LIS2DH::writeRegister(const uint8_t register_addr, const uint8_t value) {
    Wire.beginTransmission(_address); //open communication with 
    Wire.write(register_addr);  
    Wire.write(value); 
    return Wire.endTransmission(); 
}

/**
 * Write a 16b register
 */
bool LIS2DH::writeRegisters(const uint8_t msb_register, const uint8_t msb_value, const uint8_t lsb_register, const uint8_t lsb_value) { 
    //send write call to sensor address
    //send register address to sensor
    //send value to register
    bool msb_bool, lsb_bool;
    msb_bool = writeRegister(msb_register, msb_value);
    lsb_bool = writeRegister(lsb_register, lsb_value);
    return msb_bool & lsb_bool; 
}

/**
 * Change one bit of the given register
 * when value is true, the bit is forced to 1
 * when value is false, the bit is forced to 0
 */
bool LIS2DH::writeMaskedRegister8(const uint8_t register_addr, const uint8_t mask, const bool value) {
    uint8_t data = readRegister(register_addr);
    uint8_t combo;
    if(value) {
        combo = (mask | data);
    } else {
        combo = ((~mask) & data);
    }
    return writeRegister(register_addr, combo);
}

/**
 * Change a register content. The mask is applied to the value. 
 * The conten of the register is read then clear before
 * the value is applied.
 * Value is not shift
 */
bool LIS2DH::writeMaskedRegisterI(const int register_addr, const int mask, const int value) {
    uint8_t data = readRegister(register_addr);
    uint8_t masked_value = (( data & ~mask) | (mask & value)); 
    return writeRegister(register_addr, masked_value);
}

// -----------------------------------------------------
// Read to LIS2DH
// -----------------------------------------------------

/**
 * Read 8bits of data from the chip, return the value
 */
uint8_t LIS2DH::readRegister(const uint8_t register_addr) {
    Wire.beginTransmission(_address); //open communication with 
    Wire.write(register_addr);  
    Wire.endTransmission(); 
    Wire.requestFrom(_address, (uint8_t)1);
    uint8_t v = Wire.read(); 
    return v;
}

/**
 * Read 16 bits of data by reading two different registers
 */
uint16_t LIS2DH::readRegisters(const uint8_t msb_register, const uint8_t lsb_register) {
    uint8_t msb = readRegister(msb_register);
    uint8_t lsb = readRegister(lsb_register);
    return (((int16_t)msb) << 8) | lsb;
}

/**
 * Read and mask a register from the chip
 * The returned value is not shift
 */
uint8_t LIS2DH::readMaskedRegister(const uint8_t register_addr, const uint8_t mask) {
    uint8_t data = readRegister(register_addr);
    return (data & mask);
}


// -----------------------------------------------------
// Data Access
// -----------------------------------------------------

/** Read the X axis registers
 * @see LIS2DH_OUT_X_H
 * @see LIS2DH_OUT_X_L
 */
int16_t LIS2DH::getAxisX(void) {
  return readRegisters(LIS2DH_OUT_X_H, LIS2DH_OUT_X_L);
}


/** Read the Y axis registers
 * @see LIS2DH_OUT_Y_H
 * @see LIS2DH_OUT_Y_L
 */
int16_t LIS2DH::getAxisY(void) {
  return readRegisters(LIS2DH_OUT_Y_H, LIS2DH_OUT_Y_L);
}

/** Read the Z axis registers
 * @see LIS2DH_OUT_Z_H
 * @see LIS2DH_OUT_Z_L
 */
int16_t LIS2DH::getAxisZ(void) {
  return readRegisters(LIS2DH_OUT_Z_H, LIS2DH_OUT_Z_L);
}

/** Read the all axis registers
 * @see getAxisZ()
 * @see getAxisY()
 * @see getAxisZ()
 */
void LIS2DH::getMotion(int16_t* ax, int16_t* ay, int16_t* az) {
    *ax = getAxisX();
    *ay = getAxisY();
    *az = getAxisZ();
}

bool LIS2DH::tempHasOverrun(void) {
    uint8_t overrun = readMaskedRegister(LIS2DH_STATUS_REG_AUX, LIS2DH_TOR_MASK);
    return (overrun != 0);
}

bool LIS2DH::tempDataAvailable(void) {
    uint8_t data = readMaskedRegister(LIS2DH_STATUS_REG_AUX, LIS2DH_TDA_MASK);
    return (data != 0);
}

uint16_t LIS2DH::getTemperature(void) {
    if(tempDataAvailable()){
        return readRegisters(LIS2DH_OUT_TEMP_H, LIS2DH_OUT_TEMP_L);
    } else {
        //if new data isn't available
        return 0;
    }
}

bool LIS2DH::whoAmI(void) {
    return (LIS2DH_I_AM_MASK == readRegister(LIS2DH_WHO_AM_I));
}

bool LIS2DH::getTempEnabled(void) {
    return (readMaskedRegister(LIS2DH_TEMP_CFG_REG, LIS2DH_TEMP_EN_MASK) != 0);
}

bool LIS2DH::setTempEnabled(bool enable) {
    return writeRegister(LIS2DH_TEMP_CFG_REG, enable ? LIS2DH_TEMP_EN_MASK : 0);
}

uint8_t LIS2DH::getDataRate(void) {
    return readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK);
}

bool LIS2DH::setDataRate(uint8_t rate) {
    if(rate > 9) {
        return 0;
    }
    uint8_t data_rate = rate << 4;
    return writeMaskedRegisterI(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK, data_rate);
}


bool LIS2DH::enableLowPower(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, true);
}


bool LIS2DH::disableLowPower(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, false);
}


bool LIS2DH::isLowPowerEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK) != 0);
}

bool LIS2DH::enableAxisX(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_X_EN_MASK, true);
}

bool LIS2DH::disableAxisX(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_X_EN_MASK, false);
}

bool LIS2DH::isXAxisEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_X_EN_MASK) != 0);
}

bool LIS2DH::enableAxisY(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Y_EN_MASK, true);
}

bool LIS2DH::disableAxisY(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Y_EN_MASK, false);
}

bool LIS2DH::isYAxisEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_Y_EN_MASK) != 0);
}

bool LIS2DH::enableAxisZ(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Z_EN_MASK, true);
}

bool LIS2DH::disableAxisZ(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Z_EN_MASK, false);
}

bool LIS2DH::isZAxisEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_Z_EN_MASK) != 0);
}

bool LIS2DH::enableAxisXYZ(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_XYZ_EN_MASK, true);
}

bool LIS2DH::disableAxisXYZ(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_XYZ_EN_MASK, false);
}

bool LIS2DH::getHPFilterMode(uint8_t mode) {
    return readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK);
}

bool LIS2DH::setHPFilterMode(uint8_t mode) {
    if(mode > 3) {
        return 0;
    }
    uint8_t filter_mode = mode << 6;
    return writeMaskedRegisterI(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK, filter_mode);
}

//FDS functions

bool LIS2DH::EnableHPClick(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK, true);
}

bool LIS2DH::disableHPClick(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK, false);
}

bool LIS2DH::isHPClickEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK) != 0);
}

bool LIS2DH::EnableHPIS1(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIS1_MASK, true);
}

bool LIS2DH::disableHPIS1(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIS1_MASK, false);
}

bool LIS2DH::isHPIS1Enabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPIS1_MASK) != 0);
}

bool LIS2DH::EnableHPIS2(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIS2_MASK, true);
}

bool LIS2DH::disableHPIS2(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIS2_MASK, false);
}

bool LIS2DH::isHPIS2Enabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPIS2_MASK) != 0);
}

//bool LIS2DH::(void) {
//    return 0;
//}
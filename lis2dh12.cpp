/** Based on ST MicroElectronics LIS2DH datasheet http://www.st.com/web/en/resource/technical/document/datasheet/DM00042751.pdf
* 30/09/2014 by Conor Forde <me@conorforde.com>
* Updates should be available at https://github.com/Snowda/LIS2DH
*
* Changelog:
*     ... - ongoing development release
*     ... - May 2018 - Update by Disk91 / Paul Pinault to make it working
*           maintained at https://github.com/disk91/LIS2DH
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
    bool ret = true;
    Wire.begin(); 
    if ( this->whoAmI() ) {
      // connexion success
      // set a default configuration with 10Hz - Low Power ( 8b resolution )
      ret &= this->setDataRate(LIS2DH_ODR_10HZ);
      ret &= this->setResolutionMode(LIS2DH_RESOLUTION_8B);
      ret &= this->setAccelerationScale(LIS2DH_FS_SCALE_2G);

      // shutdown interrupt
      ret &= this->disableAllInterrupt();

      // enable all the axis
      ret &= this->enableAxisXYZ();
      ret &= this->setFiFoMode(LIS2DH_FM_BYPASS);

      // set highpass filter (Reset when reading xhlREFERENCE register)
      // activate High Pass filter on Output and Int1
      ret &= this->setHPFilterMode(LIS2DH_HPM_NORMAL_RESET);
      ret &= this->setHPFilterCutOff(LIS2DH_HPCF_ODR_50); // 0.2Hz high pass Fcut
      ret &= this->EnableHPFDS();
      ret &= this->EnableHPIA1();
      ret &= this->EnableHPClick();


      
    } else return false;
        
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


/** Read the X axis registers ( 8bit mode)
 * @see LIS2DH_OUT_X_L
 */
int8_t LIS2DH::getAxisX_LR(void) {
  return readRegister(LIS2DH_OUT_X_L);
}


/** Read the Y axis registers ( 8bit mode)
 * @see LIS2DH_OUT_Y_L
 */
int8_t LIS2DH::getAxisY_LR(void) {
  return readRegister(LIS2DH_OUT_Y_L);
}

/** Read the Z axis registers ( 8bit mode)
 * @see LIS2DH_OUT_Z_L
 */
int8_t LIS2DH::getAxisZ_LR(void) {
  return readRegister(LIS2DH_OUT_Z_L);
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

void LIS2DH::getMotion_LR(int8_t* ax, int8_t* ay, int8_t* az) {
    *ax = getAxisX_LR();
    *ay = getAxisY_LR();
    *az = getAxisZ_LR();
}

/**
 * Return the accelaration in mg taking into account
 * the given
 * - Resolution ( 8,10,12 bits)
 * - Scale - see LIS2DH_FS_SCALE_XG
 * The result is given in mG
 */
bool LIS2DH::getAcceleration(const uint8_t resolution, const uint8_t scale, int16_t * ax, int16_t * ay, int16_t * az) {

  if ( resolution > LIS2DH_RESOLUTION_MAXVALUE || scale > LIS2DH_FS_MAXVALUE ) return false;

  int16_t maxValue;
  int32_t x,y,z;
  bool    ret = true;
  switch ( resolution ) {
     case LIS2DH_RESOLUTION_8B:
          int8_t _x,_y,_z;
          this->getMotion_LR(&_x,&_y,&_z);
          x=_x;
          y=_y;
          z=_z;
          maxValue = 256;
          break;
    case LIS2DH_RESOLUTION_10B:
          int16_t __x,__y,__z;
          this->getMotion(&__x,&__y,&__z);
          x=__x;
          y=__y;
          z=__z;
          maxValue = 1024;
          break;
    case LIS2DH_RESOLUTION_12B:
          this->getMotion(&__x,&__y,&__z);
          x=__x;
          y=__y;
          z=__z;
          maxValue = 4096;
          break;
  }

  switch ( scale) {
    case LIS2DH_FS_SCALE_2G:
         *ax = (x*2000)/maxValue; 
         *ay = (y*2000)/maxValue; 
         *az = (z*2000)/maxValue; 
         break;
    case LIS2DH_FS_SCALE_4G:
         *ax = (x*4000)/maxValue; 
         *ay = (y*4000)/maxValue; 
         *az = (z*4000)/maxValue; 
         break;
    case LIS2DH_FS_SCALE_8G:
         *ax = (x*8000)/maxValue; 
         *ay = (y*8000)/maxValue; 
         *az = (z*8000)/maxValue; 
         break;
    case LIS2DH_FS_SCALE_16G:
         *ax = (x*16000)/maxValue; 
         *ay = (y*16000)/maxValue; 
         *az = (z*16000)/maxValue; 
         break;            
  }

  return ret;
}


/**
 * Return the accelaration force in mg over all axis
 * Given
 * - Resolution ( 8,10,12 bits)
 * - Scale - see LIS2DH_FS_SCALE_XG
 * The result is given in mG
 */
bool LIS2DH::getAccelerationForce(const uint8_t resolution, const uint8_t scale, uint16_t * force) {
  if ( resolution > LIS2DH_RESOLUTION_MAXVALUE || scale > LIS2DH_FS_MAXVALUE ) return false;
  int16_t x,y,z;
  bool ret = this->getAcceleration(resolution, scale,&x,&y,&z);
  uint32_t _force = (int32_t)x*x + (int32_t)y*y + (int32_t)z*z; 
  _force = sqrt(_force);

  *force = (uint16_t)_force;

  return ret;
}


// ======= Temperature

/**
 * Temperature is 8bit usually but it switch to 10bits if LPen bit in CTRL_REG1 is 
 * cleared (high resolution mode). In this mode the way to decode the temperature
 * is not documented in the datasheet. Test and function modification will be needed
 * ...Temperature refresh is ODR.
 * @TODO ...
 */

bool LIS2DH::getTempEnabled(void) {
    return (readMaskedRegister(LIS2DH_TEMP_CFG_REG, LIS2DH_TEMP_EN_MASK) != 0);
}

bool LIS2DH::setTempEnabled(bool enable) {
    return writeRegister(LIS2DH_TEMP_CFG_REG, enable ? LIS2DH_TEMP_EN_MASK : 0);
}

uint16_t LIS2DH::getTemperature(void) {
    if(tempDataAvailable()){
        return readRegisters(LIS2DH_OUT_TEMP_H, LIS2DH_OUT_TEMP_L);
    } else {
        //if new data isn't available
        return 0;
    }
}

bool LIS2DH::tempHasOverrun(void) {
    uint8_t overrun = readMaskedRegister(LIS2DH_STATUS_REG_AUX, LIS2DH_TOR_MASK);
    return (overrun != 0);
}

bool LIS2DH::tempDataAvailable(void) {
    uint8_t data = readMaskedRegister(LIS2DH_STATUS_REG_AUX, LIS2DH_TDA_MASK);
    return (data != 0);
}


// ======== Data Rate 

uint8_t LIS2DH::getDataRate(void) {
    return readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK);
}

bool LIS2DH::setDataRate(uint8_t data_rate) {
    if ( data_rate > LIS2DH_ODR_MAXVALUE ) return false;
    data_rate <<= LIS2DH_ODR_SHIFT;
    return writeMaskedRegisterI(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK, data_rate);
}

// ========= Power Management

bool LIS2DH::enableLowPower(void) {
    bool ret = writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, true);
    ret &= this->setHighResolutionMode(false);
    return ret;
}


bool LIS2DH::disableLowPower(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, false);
}


bool LIS2DH::isLowPowerEnabled(void) {
    return (    readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK) != 0 
             && !this->isHighResolutionMode() );
}

/**
 * Select the activity level that wake the sensor up from low_power mode.
 * This is for mode other than low_power. When this level of activity is identify
 * the sensor is waking up to the normal mode you set. Under, it goes to low-power mode
 * The unit depends on the Scale factor. you can use setSleepNWakeUpThresholdMg instead
 */
bool LIS2DH::setSleepNWakeUpThreshold(uint8_t raw) {
  if(raw > LIS2DH_ACT_THS_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_ACT_THS_SHIFT;
  return writeMaskedRegisterI(LIS2DH_ACT_THS, LIS2DH_ACT_THS_MASK, raw);
}

/**
 * Select the activity level that wake the sensor up from low-power mode in Mg. 
 * The scale is given as parameter LIS2DH_FS_SCALE_ 2/4/8/16 G
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 */
bool LIS2DH::setSleepNWakeUpThresholdMg( uint8_t mg, const uint8_t scale) {
  uint8_t raw = 0;
  if ( scale > LIS2DH_FS_MAXVALUE ) return false;

  if ( this->convertMgToRaw(&raw, mg, scale) ) {
    return this->setSleepNWakeUpThreshold(raw);
  }
  return false;
}

/**
 * Select the Activity duration for low-power to wake up and return to low-power duration
 * This is for mode other than low_power.
 * The unit depends on the ODR factor. see LIS2DH_ODR_ 1/10/25...1620 HZ 
 * You can use setSleepNWakeUpDurationMs instead
 */
bool LIS2DH::setSleepNWakeUpDuration(uint8_t raw) {
  if(raw > LIS2DH_ACT_DUR_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_ACT_DUR_SHIFT;
  return writeMaskedRegisterI(LIS2DH_ACT_DUR, LIS2DH_ACT_DUR_MASK, raw);
}

/**
 * Select the Sleep and Wake-up activity duration from a duration given in Ms
 * The duration is S = (8*rawValue+1) / ODR
 */
bool LIS2DH::setSleepNWakeUpDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr) {
  uint8_t raw = 0;
  
  // Basically this is an approximation as I did not take +1 in consideration
  ms = ms>>3;
  
  if ( convertMsToRaw(&raw, ms, odr) ) {
    if ( raw > 0 ) raw--; // this is to take -1 in consideration.
    return this->setInterruptDuration(_int,raw);
  } else {
    return false;
  }
}



// ========== Resolution mode

/**
 * Set the high Resolution mode
 * HR mode 
 */
bool LIS2DH::setHighResolutionMode(bool hr) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK,hr); 
}


bool LIS2DH::isHighResolutionMode() {
  return ( this->readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 );
}


/**
 * Set the expected acceleration resolution in bit
 * Possible option : LIS2DH_RESOLUTION_XXB (8,10,12)
 * This is changing the LowPower mode and HighResolution mode
 */
bool LIS2DH::setResolutionMode(uint8_t res) {
  if (res > LIS2DH_RESOLUTION_MAXVALUE) return false;
  bool ret;
  switch (res) {
    default:
    case LIS2DH_RESOLUTION_8B:
          return this->enableLowPower();
    case LIS2DH_RESOLUTION_10B:
          ret = this->disableLowPower();
          ret &= this->setHighResolutionMode(false);
          return ret;
    case LIS2DH_RESOLUTION_12B:
          ret = this->disableLowPower();
          ret &= this->setHighResolutionMode(true);
          return ret;
  }
}

// ========== Axis management

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

// ====== HIGH Pass filter mode
// see http://www.st.com/content/ccc/resource/technical/document/application_note/60/52/bd/69/28/f4/48/2b/DM00165265.pdf/files/DM00165265.pdf/jcr:content/translations/en.DM00165265.pdf

/*
 * Enable/Disable High Pass Filter standard output
 */
bool LIS2DH::EnableHPFDS(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_FDS_MASK, true);
}

bool LIS2DH::disableHPFDS(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_FDS_MASK, false);
}

bool LIS2DH::isHPFDSEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_FDS_MASK) != 0);
}

/**
 * High Pass filter allow to remove the continous component to only keep the
 * dynamic acceleration => basically it remove the gravity... as any stable 
 * acceleration
 */
bool LIS2DH::getHPFilterMode(uint8_t mode) {
    return readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK);
}

bool LIS2DH::setHPFilterMode(uint8_t mode) {
    if(mode > LIS2DH_HPM_MAXVALUE) {
        return false;
    }
    uint8_t filter_mode = mode << LIS2DH_HPM_SHIFT;
    return writeMaskedRegisterI(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK, filter_mode);
}

/*
 * Cut-Off Frequency
 * the reference is the ODR frequency (acquisition frequency), the Cut Off
 * frequency is this frequency divide par a given number from 9 to 400
 */
bool LIS2DH::getHPFilterCutOff(uint8_t mode) {
    return readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK);
}

bool LIS2DH::setHPFilterCutOff(uint8_t mode) {
    if(mode > LIS2DH_HPCF_MAXVALUE) {
        return false;
    }
    uint8_t fcut = mode << LIS2DH_HPCF_SHIFT;
    return writeMaskedRegisterI(LIS2DH_CTRL_REG2, LIS2DH_HPCF_MASK, fcut);
}

/*
 * Enable/Disable High Pass Filter on Click
 */
bool LIS2DH::EnableHPClick(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK, true);
}

bool LIS2DH::disableHPClick(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK, false);
}

bool LIS2DH::isHPClickEnabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK) != 0);
}

/*
 * Enable/Disable High Pass Filter on Interrupt 2
 */
bool LIS2DH::EnableHPIA2(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA2_MASK, true);
}

bool LIS2DH::disableHPIA2(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA2_MASK, false);
}

bool LIS2DH::isHPIA2Enabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPIA2_MASK) != 0);
}

/*
 * Enable/Disable High Pass Filter on Interrupt 1
 */
bool LIS2DH::EnableHPIA1(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA1_MASK, true);
}

bool LIS2DH::disableHPIA1(void) {
    return writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA1_MASK, false);
}

bool LIS2DH::isHPIA1Enabled(void) {
    return (readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPIA1_MASK) != 0);
}

// ========== INTERRUPT MANAGEMENT

/**
 * Enable / Disable an interrupt source on INT1
 */
bool LIS2DH::enableInterruptInt1(uint8_t _int) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG3,_int,true);
}

bool LIS2DH::disableInterruptInt1(uint8_t _int) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG3,_int,false);
}

/**
 * Enable / Disable an interrupt source on INT2
 */
bool LIS2DH::enableInterruptInt2(uint8_t _int) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG6,_int,true);
}

bool LIS2DH::disableInterruptInt2(uint8_t _int) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG6,_int,false);
}

/**
 * Disable all interrupts
 */
bool LIS2DH::disableAllInterrupt() {
  bool ret = writeRegister(LIS2DH_CTRL_REG3,LIS2DH_I1_INTERRUPT_NONE);
  ret &= writeMaskedRegisterI(LIS2DH_CTRL_REG6,LIS2DH_I2_MASK,LIS2DH_I2_INTERRUPT_NONE);
  return ret;
}

/**
 * Set the interrupt polarity
 */
bool LIS2DH::setInterruptPolarity(uint8_t polarity) {
  switch ( polarity ) {
    case HIGH:
      return this->writeMaskedRegister8(LIS2DH_CTRL_REG6,LIS2DH_INT_POLARITY,false);
    case LOW:
      return this->writeMaskedRegister8(LIS2DH_CTRL_REG6,LIS2DH_INT_POLARITY,true);
    default:
      return false;
  }
}


/**
 * Latch INterrupt 1/2 => the interrupt is only cleared when INT1_SRC / INT2_SRC register is read
 */
bool LIS2DH::enableLatchInterrupt1(bool enable) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_LIR_INT1_MASK,enable); 
}
bool LIS2DH::enableLatchInterrupt2(bool enable) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_LIR_INT2_MASK,enable); 
}

/**
 * Select trigger event trigger signal on INT1/2 
 * See LIS2DH_TR_XXX
 */
bool LIS2DH::triggerSelect(uint8_t triggerMode) {
  if(triggerMode > LIS2DH_TR_MAXVALUE) {
        return false;
  }
  triggerMode = triggerMode << LIS2DH_TR_SHIFT;
  return writeMaskedRegisterI(LIS2DH_FIFO_CTRL_REG, LIS2DH_TR_MASK, triggerMode);
}

/**
 * Select the interruption mode
 * See mode : LIS2DH_INT_MODE_XXX
 * Default mode is OR
 * Give the interrupt 1 for INT1 and 2 for INT2
 */
bool LIS2DH::intWorkingMode(uint8_t _int, uint8_t _mode) {
  if(_mode > LIS2DH_INT_MODE_MAXVALUE) {
        return false;
  }
  _mode = _mode << LIS2DH_INT_MODE_SHIFT;
  if ( _int > 2 || _int < 1 ) return false;  
  switch (_int) {
    case 1 : 
        return writeMaskedRegisterI(LIS2DH_INT1_CFG, LIS2DH_INT_MODE_MASK, _mode);
    case 2 :
        return writeMaskedRegisterI(LIS2DH_INT2_CFG, LIS2DH_INT_MODE_MASK, _mode);
  }
}

/**
 * Enable the interrupt events. See list 
 * It is possible to activate multiple interrupt event by adding them with | operator
 * See LIS2DH_INTEVENT_XX possible events
 * Give the interrupt 1 for INT1 and 2 for INT2
 */
bool LIS2DH::enableInterruptEvent(uint8_t _int, uint8_t _intEvent) {
  if(_intEvent > LIS2DH_INTEVENT_MAXVALUE) {
        return false;
  }
  _intEvent = _intEvent << LIS2DH_INTEVENT_SHIFT;
  if ( _int > 2 || _int < 1 ) return false;  
  switch (_int) {
    case 1 : 
        return writeMaskedRegisterI(LIS2DH_INT1_CFG, LIS2DH_INTEVENT_MASK, _intEvent);
    case 2 :
        return writeMaskedRegisterI(LIS2DH_INT2_CFG, LIS2DH_INTEVENT_MASK, _intEvent);
  }
}

/**
 * Get interrupt status
 * Give the interrupt 1 for INT1 and 2 for INT2 as parameter
 */
bool LIS2DH::isInterruptFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_INT_IA_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_INT_IA_MASK) != 0); 
  }
}

bool LIS2DH::isInterruptZHighFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_ZH_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_ZH_MASK) != 0); 
  }
}

bool LIS2DH::isInterruptZLowFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_ZL_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_ZL_MASK) != 0); 
  }
}

bool LIS2DH::isInterruptYHighFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_YH_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_YH_MASK) != 0); 
  }
}

bool LIS2DH::isInterruptYLowFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_YL_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_YL_MASK) != 0); 
  }
}

bool LIS2DH::isInterruptXHighFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_XH_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_XH_MASK) != 0); 
  }
}

bool LIS2DH::isInterruptXLowFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return false; 
  switch(_int) {
    case 1: return (readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_XL_MASK) != 0);
    case 2: return (readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_XL_MASK) != 0); 
  }
}

/**
 * Set the interrupt Threshold on selected axis
 * his value is used for High and Low value comparison on any axis to generate an interruption
 * This function write raw value. You can use the setInterruptThresholdMg function to set it in mG
 */
bool LIS2DH::setInterruptThreshold(uint8_t _int, uint8_t raw) {
  if(raw > LIS2DH_THS_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_THS_SHIFT;
  if ( _int > 2 || _int < 1 ) return false;  
  switch (_int) {
    case 1 : 
        return writeMaskedRegisterI(LIS2DH_INT1_THS, LIS2DH_THS_MASK, raw);
    case 2 :
        return writeMaskedRegisterI(LIS2DH_INT2_THS, LIS2DH_THS_MASK, raw);
  }
}

/**
 * Set the interrupt Threshold for selected axis in Mg. The scale is given as parameter
 * LIS2DH_FS_SCALE_ 2/4/8/16 G
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 */
bool LIS2DH::setInterruptThresholdMg(uint8_t _int, uint8_t mg, const uint8_t scale) {
  uint8_t raw = 0;
  if ( scale > LIS2DH_FS_MAXVALUE ) return false;

  if ( this->convertMgToRaw(&raw, mg, scale) ) {
    return this->setInterruptThreshold(_int,raw);
  }
  return false;
}

/**
 * Duration of the interrupt, the duration is function
 * of ODR value -> 1 lsb = 1/ODR
 */
bool LIS2DH::setInterruptDuration(uint8_t _int, uint8_t raw) {
  if(raw > LIS2DH_DUR_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_DUR_SHIFT;
  if ( _int > 2 || _int < 1 ) return false;  
  switch (_int) {
    case 1 : 
        return writeMaskedRegisterI(LIS2DH_INT1_DURATION, LIS2DH_D_MASK, raw);
    case 2 :
        return writeMaskedRegisterI(LIS2DH_INT2_DURATION, LIS2DH_D_MASK, raw);
  }
}

/**
 * Set the interrupt duration in Ms. The ODR value is given as parameter
 * LIS2DH_ODR_ 1/10/25...1620 HZ
 * step is 1/ODR
 */
bool LIS2DH::setInterruptDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr) {
  uint8_t raw = 0;
  if ( convertMsToRaw(&raw, ms, odr) ) {
    return this->setInterruptDuration(_int,raw);
  } else {
    return false;
  }
}


// ========== Misc settings

/**
 * Set little / big endian
 * Only valid when HR mode is on
 */
bool LIS2DH::setLittleEndian() {
  if ( this->readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 ) {
    return this->writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BLE_MASK,false);    
  }
  return false;
}

bool LIS2DH::setBitEndian() {
  if ( this->readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 ) {
    return this->writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BLE_MASK,true);    
  }
  return false;
}

/** 
 *  Select a Block Data contunious update or an update only once the MSB & LSB has been read
 */
bool LIS2DH::setContinuousUpdate(bool continuous) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BDU_MASK,~continuous); 
}

/**
 * Set the accelerations scale
 */
bool LIS2DH::setAccelerationScale(uint8_t scale) {
    if(scale > LIS2DH_FS_MAXVALUE) {
        return false;
    }
    scale = scale << LIS2DH_FS_SHIFT;
    return writeMaskedRegisterI(LIS2DH_CTRL_REG4, LIS2DH_FS_MASK, scale);
}


/** 
 *  Reboot memory content
 */
bool LIS2DH::reboot() {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_BOOT_MASK,true); 
}

/**
 * Return true if the WHOAMI register returned th expected Value
 * 0x33
 */
bool LIS2DH::whoAmI(void) {
    return (LIS2DH_I_AM_VALUE == readRegister(LIS2DH_WHO_AM_I));
}

/**
 * Enable / Disable Fifo
 */
bool LIS2DH::enableFifo(bool enable) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_FIFO_EN_MASK,enable); 
}


// ======== REFERENCE used for interrupt generation

bool LIS2DH::setReference(uint8_t ref) {
  return this->writeRegister(LIS2DH_REFERENCE,ref);
}

// ========= STATUS
/**
 * Return the Data status bit filed.
 * See LIS2DH_STATUS_XXXX for different possible status
 */
uint8_t LIS2DH::getDataStatus() {
  return this->readRegister(LIS2DH_STATUS_REG2);
}

// ========= FIFO CTRL

/**
 * Select the FIFO working mode
 * see LIS2DH_FM_XXX possible mode
 * 
 */
bool LIS2DH::setFiFoMode(uint8_t fifoMode) {
    if(fifoMode > LIS2DH_FM_MAXVALUE) {
        return false;
    }
    fifoMode = fifoMode << LIS2DH_FM_SHIFT;
    return writeMaskedRegisterI(LIS2DH_FIFO_CTRL_REG, LIS2DH_FM_MASK, fifoMode);
}

/**
 * Set the Fifo Threshold : as soon as this level of data is filled in FiFo the
 * Threshold event is triggered for reading
 * This is used for reading FiFo before it overrun.
 * The value is from 0 to 31.
 */
bool LIS2DH::setFiFoThreshold(uint8_t threshold) {
  if ( threshold > LIS2DH_FTH_MAXVALUE ) return false;
  threshold <<= LIS2DH_FTH_SHIFT;
  return writeMaskedRegisterI(LIS2DH_FIFO_CTRL_REG, LIS2DH_FTH_MASK, threshold);
}

/**
 * Return true when FIFO watermark level exceeded
 */
bool LIS2DH::isFiFoWatermarkExceeded() {
  return ( this->readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_WTM_MASK) > 0 ); 
}

/**
 * Return true when the FIFO is full => 32 unread samples
 */
bool LIS2DH::isFiFoFull() {
  return ( this->readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_OVRN_FIFO_MASK) > 0 ); 
}

/**
 * Return true when the FIFO is empty 
 */
bool LIS2DH::isFiFoEmpty() {
  return ( this->readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_EMPTY_MASK) > 0 ); 
}

/**
 * Return the number of sample actually in the FiFo
 */
uint8_t LIS2DH::getFiFoSize() {
  return this->readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_FSS_MASK);
}

// ======== CLICK Management

/**
 * Enable the CLICK function interrupt events. See list 
 * It is possible to activate multiple interrupt event by adding them with | operator
 * See LIS2DH_CLICEVENT_XXX possible events
 * Give the interrupt 1 for INT1 and 2 for INT2
 */
bool LIS2DH::enableInterruptEvent(uint8_t _intEvent) {
  if(_intEvent > LIS2DH_CLICEVENT_MAXVALUE) {
        return false;
  }
  _intEvent = _intEvent << LIS2DH_CLICEVENT_SHIFT;
  return writeMaskedRegisterI(LIS2DH_CLICK_CFG, LIS2DH_CLICEVENT_MASK, _intEvent);
}


/**
 * Get clic interrupt status
 * Get the global information for all the interrupts
 */
bool LIS2DH::isClickInterruptFired() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_CLK_IA_MASK) != 0);
}
bool LIS2DH::isDoubleClickFired() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_DCLICK_MASK) != 0);
}
bool LIS2DH::isSimpleClickFired() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_SCLICK_MASK) != 0);
}
bool LIS2DH::isClickFiredOnZ() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_Z_CLICK_MASK) != 0);
}
bool LIS2DH::isClickFiredOnY() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_Y_CLICK_MASK) != 0);
}
bool LIS2DH::isClickFiredOnX() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_X_CLICK_MASK) != 0);
}
bool LIS2DH::isSignClickFired() {
  return (readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_SIGN_MASK) != 0);
}

/**
 * From the clic interrupt status, determine the type of pending
 * clic interrupt
 * see LIS2DH_CLIC_XXXX for the possible clic. Multiple clic can be
 * reported. 1bit per clic
 * By reading the register, the insterrup will fall depends on click_ths configuration
 */
uint16_t LIS2DH::getClickStatus() {
  uint8_t clic=readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_CLICK_SRC_MASK);
  uint16_t ret = LIS2DH_CLIC_NONE;
  if ( (clic & LIS2DH_CLK_IA_MASK) > 0 ) {
    if ( (clic & LIS2DH_X_CLICK_MASK) > 0 ) {
      if ( (clic & LIS2DH_SCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_SIN_X_NEG;
        } else{
          ret |= LIS2DH_CLIC_SIN_X_POS;
        }
      }
      if ( (clic & LIS2DH_DCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_DBL_X_NEG;
        } else{
          ret |= LIS2DH_CLIC_DBL_X_POS;
        }
      }
    }

    if ( (clic & LIS2DH_Y_CLICK_MASK) > 0 ) {
      if ( (clic & LIS2DH_SCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_SIN_Y_NEG;
        } else{
          ret |= LIS2DH_CLIC_SIN_Y_POS;
        }
      }
      if ( (clic & LIS2DH_DCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_DBL_Y_NEG;
        } else{
          ret |= LIS2DH_CLIC_DBL_Y_POS;
        }
      }
    }

    if ( (clic & LIS2DH_Z_CLICK_MASK) > 0 ) {
      if ( (clic & LIS2DH_SCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_SIN_Z_NEG;
        } else{
          ret |= LIS2DH_CLIC_SIN_Z_POS;
        }
      }
      if ( (clic & LIS2DH_DCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_DBL_Z_NEG;
        } else{
          ret |= LIS2DH_CLIC_DBL_Z_POS;
        }
      }
    }

  }
  return LIS2DH_CLIC_NONE;
}



/**
 * Set the Click threshold. The default value is zero.
 * The pretty poor documentation do not gives unit for 
 * this paramtere.
 * We could imagine it is the same as interrupt threshold :
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G 
 * but this needs to be confirmed.
 */
bool LIS2DH::setClickThreshold(uint8_t ths) {
  if(ths > LIS2DH_CLK_THS_MAXVALUE) {
        return false;
  }
  ths = ths << LIS2DH_CLK_THS_SHIFT;
  return writeMaskedRegisterI(LIS2DH_CLICK_THS, LIS2DH_CLK_THS_MASK, ths);
}

bool LIS2DH::setClickThresholdMg(uint16_t mg, const uint8_t scale) {
  // Not having information of the threshold unit it is not possible 
  // to make this function working correctly.
  // Any information is welcome to correctly implement it.
  // Actally assuming it works like the other functions...
  uint8_t raw = 0;
  if ( scale > LIS2DH_FS_MAXVALUE ) return false;

  if ( this->convertMgToRaw(&raw, mg, scale) ) {
    return this->setClickThreshold(raw);
  }
  return false;
}

/**
 * Set the Interrupt mode for click
 * It can be LIS2DH_CLK_INTDUR_UNTILREAD ( interrupt pending unteil CLICK_SRC register read )
 * or LIS2DH_CLK_INTDUR_LATWINDOW ( interrupt cancel afet TIME_LATENCY duration
 */
bool LIS2DH::setClickInterruptMode(uint8_t _mode) {
  if(_mode > LIS2DH_CLK_INTDUR_MAXVALUE) {
        return false;
  }
  _mode = _mode << LIS2DH_CLK_INTDUR_SHIFT;
  return writeMaskedRegisterI(LIS2DH_CLICK_THS, LIS2DH_CLK_INTDUR_MASK, _mode);
}

/**
 * Set the Time limit -> due to a lack of documentation in the datasheet and application notes
 * It is hard to say what is this register and what is the unit to use in the register 
 * (Question to ST : how can you such bad for documentation ?)
 * I assume time limit is the maximum time to detect a clic or double clic, but starting from ?
 */
bool LIS2DH::setClickTimeLimit(uint8_t raw){
  if(raw > LIS2DH_TLI_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_TLI_SHIFT;
  return writeMaskedRegisterI(LIS2DH_TIME_LIMIT, LIS2DH_TLI_MASK, raw);
}

/**
 * Set the Time latency for click -> due to a lack of documentation in the datasheet and application notes
 * It is hard to say what is this register and what is the unit to use in the register 
 * (Question to ST : how can you such bad for documentation ?)
 * I do not know what this timer do
 */
bool LIS2DH::setClickTimeLatency(uint8_t raw){
  if(raw > LIS2DH_TIME_LATENCY_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_TIME_LATENCY_SHIFT;
  return writeMaskedRegisterI(LIS2DH_TIME_LATENCY, LIS2DH_TIME_LATENCY_MASK, raw);
} 


/**
 * Set the Time Window for click -> due to a lack of documentation in the datasheet and application notes
 * It is hard to say what is this register and what is the unit to use in the register 
 * (Question to ST : how can you such bad for documentation ?)
 * I do not know what this timer do
 */
bool LIS2DH::setClickTimeWindow(uint8_t raw){
  if(raw > LIS2DH_TIME_WINDOW_MAXVALUE) {
        return false;
  }
  raw = raw << LIS2DH_TIME_WINDOW_SHIFT;
  return writeMaskedRegisterI(LIS2DH_TIME_WINDOW, LIS2DH_TIME_WINDOW_MASK, raw);
} 


// ======== 4D/6D Device Positionning

/**
 * Honestly even if the documentation is claiming this feature exists in the device, 
 * for real, there is no information on How you can get it from the registers.
 * If some developpers find a way to use it, let me know.
 */


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
// Read from LIS2DH
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

// ----------------------------------------------------
// Internal common converters
// ----------------------------------------------------

/**
 * From a scale LIS2DH_FS_SCALE_ 2/4/8/16 G value, return
 * the Raw equivalent value to the given mG
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 * The result is return to _raw 
 * Returns false in case of convertion error
 */
bool LIS2DH::convertMgToRaw(uint8_t * _raw, uint16_t mg, uint8_t scale) {
  uint16_t raw;
  switch ( scale ) {
    case LIS2DH_FS_SCALE_2G:
        raw = mg / 16;
        if ( raw == 0 && mg > 0 ) return false;
        break;
    case LIS2DH_FS_SCALE_4G:
        raw = mg / 32;
        if ( raw == 0 && mg > 0 ) return false;
        break;
    case LIS2DH_FS_SCALE_8G:
        raw = mg / 62;
        if ( raw == 0 && mg > 0 ) return false;
        break;
    case LIS2DH_FS_SCALE_16G:
        raw = mg / 186;
        if ( raw == 0 && mg > 0 ) return false;
        break;
  }
  if ( raw < 256 ) {
    *_raw = (uint8_t)raw;
    return true;
  } else {
    return false;
  }
}

/**
 * Convert a duration in Ms to a raw duration based on the ODR value given as parameter
 * LIS2DH_ODR_ 1/10/25...1620 HZ
 * step is 1/ODR
 */
bool LIS2DH::convertMsToRaw(uint8_t * _raw, uint32_t ms, const uint8_t odr) {
  uint8_t raw = 0;
  if ( odr >= LIS2DH_ODR_MAXVALUE ) return false;
  
  switch ( odr ) {
    case LIS2DH_ODR_1HZ:
        raw = (uint8_t)(ms / 1000);
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_10HZ:
        raw = (uint8_t)(ms / 100);
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_25HZ:
        raw = (uint8_t)(ms / 40);
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_50HZ:
        raw = (uint8_t)(ms / 20);
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_100HZ:
        raw = (uint8_t)(ms / 10);
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_200HZ:
        raw = (uint8_t)(ms / 5);
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_400HZ:
        raw = (uint8_t)((ms * 2) / 5) ;
        if ( raw == 0 && ms > 0 ) return false;
        break;
    case LIS2DH_ODR_1620HZ:
        raw = (uint8_t)((ms * 1000) / 617);
        if ( raw == 0 && ms > 0 ) return false;
        break;
  }
  if ( raw < 256 ) {
    *_raw = (uint8_t)raw;
    return true;
  } else {
    return false;
  }
}
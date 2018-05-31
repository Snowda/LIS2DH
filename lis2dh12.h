/** Based on ST MicroElectronics LIS2DH datasheet http://www.st.com/web/en/resource/technical/document/datasheet/DM00042751.pdf
* 18/06/2014 by Conor Forde <me@conorforde.com>
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

#ifndef _LIS2DH_H_
#define _LIS2DH_H_

//Registers
#define LIS2DH_STATUS_REG_AUX   0x07
#define LIS2DH_OUT_TEMP_L     0x0C
#define LIS2DH_OUT_TEMP_H     0x0D
#define LIS2DH_INT_COUNTER_REG  0x0E
#define LIS2DH_WHO_AM_I     0x0F
#define LIS2DH_TEMP_CFG_REG   0x1F
#define LIS2DH_CTRL_REG1    0x20
#define LIS2DH_CTRL_REG2    0x21
#define LIS2DH_CTRL_REG3    0x22
#define LIS2DH_CTRL_REG4    0x23
#define LIS2DH_CTRL_REG5    0x24
#define LIS2DH_CTRL_REG6    0x25
#define LIS2DH_REFERENCE    0x26
#define LIS2DH_STATUS_REG2    0x27
#define LIS2DH_OUT_X_L      0x28
#define LIS2DH_OUT_X_H      0x29
#define LIS2DH_OUT_Y_L      0x2A
#define LIS2DH_OUT_Y_H      0x2B
#define LIS2DH_OUT_Z_L      0x2C
#define LIS2DH_OUT_Z_H      0x2D
#define LIS2DH_FIFO_CTRL_REG  0x2E
#define LIS2DH_FIFO_SRC_REG   0x2F
#define LIS2DH_INT1_CFG     0x30
#define LIS2DH_INT1_SOURCE    0x31
#define LIS2DH_INT1_THS     0x32
#define LIS2DH_INT1_DURATION  0x33
#define LIS2DH_INT2_CFG      0x34
#define LIS2DH_INT2_SOURCE    0x35
#define LIS2DH_INT2_THS     0x36
#define LIS2DH_INT2_DURATION  0x37
#define LIS2DH_CLICK_CFG    0x38
#define LIS2DH_CLICK_SRC    0x39
#define LIS2DH_CLICK_THS    0x3A
#define LIS2DH_TIME_LIMIT     0x3B
#define LIS2DH_TIME_LATENCY   0x3C
#define LIS2DH_TIME_WINDOW    0x3D
#define LIS2DH_ACT_THS      0x3E
#define LIS2DH_ACT_DUR      0x3F

//Register Masks

//STATUS_AUX_REG masks
#define LIS2DH_TOR_MASK     0x40
#define LIS2DH_TDA_MASK     0x04

//INT_COUNTER masks
//what goes here?

//WHO_AM_I masks
#define LIS2DH_I_AM_VALUE   0x33

// TEMP_CFG_REG masks
#define LIS2DH_TEMP_EN_MASK   0xC0

// CTRL_REG1 masks
#define LIS2DH_ODR_MASK     0xF0
#define LIS2DH_LPEN_MASK    0x08
#define LIS2DH_Z_EN_MASK    0x04
#define LIS2DH_Y_EN_MASK    0x02
#define LIS2DH_X_EN_MASK    0x01
#define LIS2DH_XYZ_EN_MASK  0x07

#define LIS2DH_ODR_SHIFT      4
#define LIS2DH_ODR_POWER_DOWN 0x00
#define LIS2DH_ODR_1HZ        0x01
#define LIS2DH_ODR_10HZ       0x02
#define LIS2DH_ODR_25HZ       0x03
#define LIS2DH_ODR_50HZ       0x04
#define LIS2DH_ODR_100HZ      0x05
#define LIS2DH_ODR_200HZ      0x06
#define LIS2DH_ODR_400HZ      0x07
#define LIS2DH_ODR_1620HZ     0x08
#define LIS2DH_ODR_1344HZ     0x09
#define LIS2DH_ODR_5376HZ     0x09
#define LIS2DH_ODR_MAXVALUE   0x09


// CTRL_REG2 masks
#define LIS2DH_HPM_MASK         0xC0
#define LIS2DH_HPCF_MASK        0x30
#define LIS2DH_FDS_MASK         0x08
#define LIS2DH_HPCLICK_MASK     0x04
#define LIS2DH_HPIA2_MASK       0x02        // Apply filtering on interrupt 2
#define LIS2DH_HPIA1_MASK       0x01        // Apply filtering on interrupt 1

#define LIS2DH_HPM_SHIFT          6
#define LIS2DH_HPM_NORMAL_RESET   0x00      // In this mode - when reading on of the XL/XH_REFERENCE register the current acceleration is reset on the corresponding axe (manuela reset)
#define LIS2DH_HPM_REFSIG         0x01      // In this mode acceleration is the difference with the XL/XH_REFERENCE content for each axis
#define LIS2DH_HPM_NORMAL2        0x02      // In this mode I assume we have no filtering
#define LIS2DH_HPM_AUTORESET      0x03      // In this mode the interrupt event will reset the filter
#define LIS2DH_HPM_MAXVALUE       0x03

#define LIS2DH_HPCF_SHIFT         4
#define LIS2DH_HPCF_ODR_50        0x00      // F cut = ODR Freq / 50
#define LIS2DH_HPCF_ODR_100       0x01
#define LIS2DH_HPCF_ODR_9         0x02
#define LIS2DH_HPCF_ODR_400       0x03
#define LIS2DH_HPCF_MAXVALUE      0x03

// CTRL_REG3 masks
#define LIS2DH_I1_CLICK           0x80      // Interrupt on click on INT1
#define LIS2DH_I1_IA1             0x40      // Interrupt from IA1 on INT1
#define LIS2DH_I1_IA2             0x20      // Interrupt from IA2 on INT1
#define LIS3DH_I1_ZYXDA           0x10      // Any Data axis on INT1
#define LIS2DH_I1_WTM             0x04      // FiFo Watermark on INT1
#define LIS2DH_I1_OVERRUN         0x02      // FiFo Overrun on INT1
#define LIS2DH_I1_INTERRUPT_NONE  0x00

// CTRL_REG6 masks
#define LIS2DH_I2_MASK            0xF8       // Mask for interrupt
#define LIS2DH_I2_CLICK           0x80       // Click interrupt
#define LIS2DH_I2_IA1             0x40       // Interrupt 1 function
#define LIS2DH_I2_IA2             0x20       // Interrupt 2 function
#define LIS2DH_I2_BOOT            0x10       // Boot interrupt
#define LIS2DH_I2_ACTIVITY        0x08       // Activity interrupt
#define LIS2DH_I2_INTERRUPT_NONE  0x00
#define LIS2DH_INT_POLARITY       0x02        // Interupt polarity => 0 active high / 1 active low



// CTRL_REG4 masks
#define LIS2DH_BDU_MASK     0x80
#define LIS2DH_BLE_MASK     0x40
#define LIS2DH_FS_MASK      0x30
#define LIS2DH_HR_MASK      0x08
#define LIS2DH_ST_MASK      0x06
#define LIS2DH_SIM_MASK     0x01

#define LIS2DH_FS_SHIFT     4
#define LIS2DH_FS_SCALE_2G  0x00
#define LIS2DH_FS_SCALE_4G  0x01
#define LIS2DH_FS_SCALE_8G  0x02
#define LIS2DH_FS_SCALE_16G 0x03
#define LIS2DH_FS_MAXVALUE  0x03

#define LIS2DH_RESOLUTION_8B        0x01      // Different resolution mode => Low Power
#define LIS2DH_RESOLUTION_10B       0x02      // Normal mode (10b)
#define LIS2DH_RESOLUTION_12B       0x03      // High Resolution mode (12b)
#define LIS2DH_RESOLUTION_MAXVALUE  0x03


// CTRL_REG5 masks
#define LIS2DH_BOOT_MASK      0x80
#define LIS2DH_FIFO_EN_MASK   0x40
#define LIS2DH_LIR_INT1_MASK  0x08
#define LIS2DH_D4D_INT1_MASK  0x04
#define LIS2DH_LIR_INT2_MASK  0x02
#define LIS2DH_D4D_INT2_MASK  0x01


// REF masks
// none

// STATUS_REG masks
#define LIS2DH_STATUS_ZYXOR     0x80      // X, Y, Z data overrun => a new set of data has overwritten the previous set
#define LIS2DH_STATUS_ZOR       0x40      // Z overrun
#define LIS2DH_STATUS_YOR       0x20      // Y overrun
#define LIS2DH_STATUS_XOR       0x10      // X overrun
#define LIS2DH_STATUS_ZYXDA     0x08      // X, Y, Z data available => a new set of data is availbale
#define LIS2DH_STATUS_ZDA       0x04      // Z data available
#define LIS2DH_STATUS_YDA       0x02      // Y data available
#define LIS2DH_STATUS_XDA       0x01      // X data available

// FIFO_CTRL_REG masks
#define LIS2DH_FM_MASK          0xC0
#define LIS2DH_TR_MASK          0x20
#define LIS2DH_FTH_MASK         0x1F

#define LIS2DH_FM_SHIFT         6
#define LIS2DH_FM_BYPASS        0x00      // No FIFO at all, the acceleration are not stored in FIFO
#define LIS2DH_FM_FIFO          0x01      // FIFO is used until being full after that no more data are added until clearing it by switching to bypass
#define LIS2DH_FM_STREAM        0x02      // FIFO is used and when full the older data are replaced by the new one.
#define LIS2DH_FM_STREAMFIFO    0x03      // In this mode the Interrupt Generator will automatically swicth the mode from STREAM to FiFo
#define LIS2DH_FM_MAXVALUE      0x03    

#define LIS2DH_TR_SHIFT         5
#define LIS2DH_TR_INT1          0x00
#define LIS2DH_TR_INT2          0x01
#define LIS2DH_TR_MAXVALUE      0x01

#define LIS2DH_FTH_SHIFT        0
#define LIS2DH_FTH_MAXVALUE     32

// FIFO_SRC_REG masks
#define LIS2DH_WTM_MASK          0x80
#define LIS2DH_OVRN_FIFO_MASK    0x40
#define LIS2DH_EMPTY_MASK        0x20
#define LIS2DH_FSS_MASK          0x1F

// INT1/2_CFG masks
#define LIS2DH_AOI_MASK         0x80
#define LIS2DH_6D_MASK          0x40
#define LIS2DH_INT_MODE_MASK    0xC0
#define LIS2DH_ZHIE_MASK        0x20
#define LIS2DH_ZLIE_MASK        0x10
#define LIS2DH_YHIE_MASK        0x08
#define LIS2DH_YLIE_MASK        0x04
#define LIS2DH_XHIE_MASK        0x02
#define LIS2DH_XLIE_MASK        0x01
#define LIS2DH_INTEVENT_MASK    0x3F


#define LIS2DH_INT_MODE_SHIFT   6
#define LIS2DH_INT_MODE_OR      0x00      // If one of the event triggers, the interrupt is fired
#define LIS2DH_INT_MODE_AND     0x02      // When all the event have triggered, the inteerupt is fired
#define LIS2DH_INT_MODE_MOV     0x01      // Movement recognition => when the orientation move from and unknown zone to a known zone, duration is ODR
#define LIS2DH_INT_MODE_DIR     0x03      // Interrupt fired when the orientation is in a known zone and stay until we stay in this zone
#define LIS2DH_INT_MODE_MAXVALUE  0x03

#define LIS2DH_INTEVENT_SHIFT   0
#define LIS2DH_INTEVENT_Z_HIGH  0x20      // Fire interrupt on Z high event of direction recognotion
#define LIS2DH_INTEVENT_Z_LOW   0x10      // Fire interrupt on Z low event of direction recognotion
#define LIS2DH_INTEVENT_Y_HIGH  0x08      // Fire interrupt on Y high event of direction recognotion
#define LIS2DH_INTEVENT_Y_LOW   0x04      // Fire interrupt on Y low event of direction recognotion
#define LIS2DH_INTEVENT_X_HIGH  0x02      // Fire interrupt on X high event of direction recognotion
#define LIS2DH_INTEVENT_X_LOW   0x01      // Fire interrupt on X low event of direction recognotion
#define LIS2DH_INTEVENT_MAXVALUE 0x3F


// INT1/2_SRC masks
#define LIS2DH_INT_IA_MASK    0x40
#define LIS2DH_ZH_MASK        0x20
#define LIS2DH_ZL_MASK        0x10
#define LIS2DH_YH_MASK        0x08
#define LIS2DH_YL_MASK        0x04
#define LIS2DH_XH_MASK        0x02
#define LIS2DH_XL_MASK        0x01

// INT1/2_THS masks
#define LIS2DH_THS_MASK       0x7F
#define LIS2DH_THS_SHIFT      0
#define LIS2DH_THS_MAXVALUE   0x7F

// INT1/2_DURATION masks
#define LIS2DH_D_MASK         0x7F
#define LIS2DH_DUR_SHIFT      0
#define LIS2DH_DUR_MAXVALUE   0x7F

// CLICK_CFG masks
#define LIS2DH_ZD_MASK      0x20
#define LIS2DH_ZS_MASK      0x10
#define LIS2DH_YD_MASK      0x08
#define LIS2DH_YS_MASK      0x04
#define LIS2DH_XD_MASK      0x02
#define LIS2DH_XS_MASK      0x01

// CLICK_SRC masks
#define LIS2DH_CLK_IA_MASK    0x40
#define LIS2DH_DCLICK_MASK    0x20
#define LIS2DH_SCLICK_MASK    0x10
#define LIS2DH_SIGN_MASK    0x08
#define LIS2DH_Z_CLICK_MASK   0x04
#define LIS2DH_Y_CLICK_MASK   0x02
#define LIS2DH_X_CLICK_MASK   0x01

// CLICK_THS masks
#define LIS2DH_CLK_THS_MASK   0x7F

// TIME_LIMIT masks
#define LIS2DH_TLI_MASK     0x7F

// TIME_LATENCY masks
// none

// TIME_WINDOW masks
// none

// ACT_THS masks
#define LIS2DH_ACTH_MASK    0x7F

// ACT_DUR masks
// None
#define LIS2DH_DEFAULT_ADDRESS  0x18

#define LIS2DH_LIS2DH_SA0_LOW     LIS2DH_DEFAULT_ADDRESS
#define LIS2DH_LIS2DH_SA0_HIGH    (LIS2DH_DEFAULT_ADDRESS+1)



class LIS2DH {
 public:
    LIS2DH(uint8_t);
    bool init(void);

    int16_t getAxisX(void);
    int16_t getAxisY(void);
    int16_t getAxisZ(void);
    void getMotion(int16_t* ax, int16_t* ay, int16_t* az);
    int8_t getAxisX_LR(void);
    int8_t getAxisY_LR(void);
    int8_t getAxisZ_LR(void);
    void getMotion_LR(int8_t* ax, int8_t* ay, int8_t* az);
    bool getAcceleration(const uint8_t resolution, const uint8_t scale, int16_t * ax, int16_t * ay, int16_t * az);
    bool getAccelerationForce(const uint8_t resolution, const uint8_t scale, uint16_t * force);

    bool tempHasOverrun(void);
    bool tempDataAvailable(void);
    uint16_t getTemperature(void);
    bool whoAmI(void);
    bool getTempEnabled(void);
    bool setTempEnabled(bool enable);
    uint8_t getDataRate(void);
    bool setDataRate(uint8_t rate);
    bool enableLowPower(void);
    bool disableLowPower(void);
    bool isLowPowerEnabled(void);
    bool enableAxisX(void);
    bool disableAxisX(void);
    bool isXAxisEnabled(void);
    bool enableAxisY(void);
    bool disableAxisY(void);
    bool isYAxisEnabled(void);
    bool enableAxisZ(void);
    bool disableAxisZ(void);
    bool isZAxisEnabled(void);
    bool getHPFilterMode(uint8_t mode);
    bool setHPFilterMode(uint8_t mode);
    bool getHPFilterCutOff(uint8_t mode);
    bool setHPFilterCutOff(uint8_t mode);
    bool EnableHPClick(void);
    bool disableHPClick(void);
    bool isHPClickEnabled(void);
    bool EnableHPIA1(void);
    bool disableHPIA1(void);
    bool isHPIA1Enabled(void);
    bool EnableHPIA2(void);
    bool disableHPIA2(void);
    bool isHPIA2Enabled(void);
    bool EnableHPFDS(void);
    bool disableHPFDS(void);
    bool isHPFDSEnabled(void);
    bool enableAxisXYZ(void);
    bool disableAxisXYZ(void);
    
    bool enableInterruptInt1(uint8_t _int);
    bool disableInterruptInt1(uint8_t _int);
    bool enableInterruptInt2(uint8_t _int);
    bool disableInterruptInt2(uint8_t _int);
    bool disableAllInterrupt();
    bool setInterruptPolarity(uint8_t polarity);
    bool triggerSelect(uint8_t triggerMode);
    bool intWorkingMode(uint8_t _int, uint8_t _mode);
    bool enableInterruptEvent(uint8_t _int, uint8_t _intEvent);
    bool isInterruptFired(uint8_t _int);
    bool isInterruptZHighFired(uint8_t _int);
    bool isInterruptZLowFired(uint8_t _int);
    bool isInterruptYHighFired(uint8_t _int);
    bool isInterruptYLowFired(uint8_t _int);
    bool isInterruptXHighFired(uint8_t _int);
    bool isInterruptXLowFired(uint8_t _int);
    bool setInterruptThreshold(uint8_t _int, uint8_t raw);
    bool setInterruptThresholdMg(uint8_t _int, uint8_t mg, const uint8_t scale);
    bool setInterruptDuration(uint8_t _int, uint8_t raw);
    bool setInterruptDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr);    

    bool setLittleEndian();
    bool setBitEndian();
    bool setContinuousUpdate(bool continuous);
    bool setAccelerationScale(uint8_t scale);
    bool setHighResolutionMode(bool hr);
    bool isHighResolutionMode();

    bool reboot();
    bool enableFifo(bool enable);
    bool enableLatchInterrupt1(bool enable);
    bool enableLatchInterrupt2(bool enable);

    bool setReference(uint8_t ref);
    uint8_t getDataStatus();
    bool setResolutionMode(uint8_t res);

    bool setFiFoMode(uint8_t fifoMode);
    bool setFiFoThreshold(uint8_t threshold);
    bool isFiFoWatermarkExceeded();
    bool isFiFoFull();
    bool isFiFoEmpty();
    uint8_t getFiFoSize();

 
 private:
    bool writeRegister(const uint8_t register_addr, const uint8_t value);
    bool writeRegisters(const uint8_t msb_register, const uint8_t msb_value, const uint8_t lsb_register, const uint8_t lsb_value);
    bool writeMaskedRegister8(const uint8_t register_addr, const uint8_t mask, const bool value);
    bool writeMaskedRegisterI(const int register_addr, const int mask, const int value);
    uint8_t  readRegister(const uint8_t register_addr);
    uint16_t readRegisters(const uint8_t msb_register, const uint8_t lsb_register);
    uint8_t  readMaskedRegister(const uint8_t register_addr, const uint8_t mask);

    uint8_t _address;
};

#endif /* _LIS2DH_H_ */
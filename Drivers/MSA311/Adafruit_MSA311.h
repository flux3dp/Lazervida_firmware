/*!
 * @file     Adafruit_MSA301.h
 */

#ifndef ADAFRUIT_MSA311_H
#define ADAFRUIT_MSA311_H

#include <stdint.h>
#include <stdbool.h>

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define MSA311_I2CADDR_DEFAULT (0x26) ///< Fixed I2C address
#define MSA311_I2CADDR_DEFAULT (0x62) ///< Fixed I2C address
/*=========================================================================*/

#define MSA311_REG_PARTID 0x01    ///< Register that contains the part ID
#define MSA311_REG_OUT_X_L 0x02   ///< Register address for X axis lower byte
#define MSA311_REG_OUT_X_H 0x03   ///< Register address for X axis higher byte
#define MSA311_REG_OUT_Y_L 0x04   ///< Register address for Y axis lower byte
#define MSA311_REG_OUT_Y_H 0x05   ///< Register address for Y axis higher byte
#define MSA311_REG_OUT_Z_L 0x06   ///< Register address for Z axis lower byte
#define MSA311_REG_OUT_Z_H 0x07   ///< Register address for Z axis higher byte
#define MSA311_REG_MOTIONINT 0x09 ///< Register address for motion interrupt
#define MSA311_REG_DATAINT 0x0A   ///< Register address for data interrupt
#define MSA311_REG_CLICKSTATUS                                                 \
  0x0B ///< Register address for click/doubleclick status
#define MSA311_REG_RESRANGE 0x0F  ///< Register address for resolution range
#define MSA311_REG_ODR 0x10       ///< Register address for data rate setting
#define MSA311_REG_POWERMODE 0x11 ///< Register address for power mode setting
#define MSA311_REG_INTSET0 0x16   ///< Register address for interrupt setting #0
#define MSA311_REG_INTSET1 0x17   ///< Register address for interrupt setting #1
#define MSA311_REG_INTMAP0 0x19   ///< Register address for interrupt map #0
#define MSA311_REG_INTMAP1 0x1A   ///< Register address for interrupt map #1
#define MSA311_REG_TAPDUR 0x2A    ///< Register address for tap duration
#define MSA311_REG_TAPTH 0x2B     ///< Register address for tap threshold

/** The accelerometer ranges */
typedef enum {
  MSA311_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
  MSA311_RANGE_4_G = 0b01,  ///< +/- 4g
  MSA311_RANGE_8_G = 0b10,  ///< +/- 8g
  MSA311_RANGE_16_G = 0b11, ///< +/- 16g
} msa311_range_t;

/** The accelerometer axes */
typedef enum {
  MSA311_AXIS_X = 0x0, ///< X axis bit
  MSA311_AXIS_Y = 0x1, ///< Y axis bit
  MSA311_AXIS_Z = 0x2, ///< Z axis bit
} msa311_axis_t;

/** The accelerometer data rate */
typedef enum {
  MSA311_DATARATE_1_HZ = 0b0000,     ///<  1 Hz
  MSA311_DATARATE_1_95_HZ = 0b0001,  ///<  1.95 Hz
  MSA311_DATARATE_3_9_HZ = 0b0010,   ///<  3.9 Hz
  MSA311_DATARATE_7_81_HZ = 0b0011,  ///<  7.81 Hz
  MSA311_DATARATE_15_63_HZ = 0b0100, ///<  15.63 Hz
  MSA311_DATARATE_31_25_HZ = 0b0101, ///<  31.25 Hz
  MSA311_DATARATE_62_5_HZ = 0b0110,  ///<  62.5 Hz
  MSA311_DATARATE_125_HZ = 0b0111,   ///<  125 Hz
  MSA311_DATARATE_250_HZ = 0b1000,   ///<  250 Hz
  MSA311_DATARATE_500_HZ = 0b1001,   ///<  500 Hz
  MSA311_DATARATE_1000_HZ = 0b1010,  ///<  1000 Hz
} msa311_dataRate_t;

/** The accelerometer bandwidth */
typedef enum {
  MSA311_BANDWIDTH_1_95_HZ = 0b0000,  ///<  1.95 Hz
  MSA311_BANDWIDTH_3_9_HZ = 0b0011,   ///<  3.9 Hz
  MSA311_BANDWIDTH_7_81_HZ = 0b0100,  ///<  7.81 Hz
  MSA311_BANDWIDTH_15_63_HZ = 0b0101, ///<  15.63 Hz
  MSA311_BANDWIDTH_31_25_HZ = 0b0110, ///<  31.25 Hz
  MSA311_BANDWIDTH_62_5_HZ = 0b0111,  ///<  62.5 Hz
  MSA311_BANDWIDTH_125_HZ = 0b1000,   ///<  125 Hz
  MSA311_BANDWIDTH_250_HZ = 0b1001,   ///<  250 Hz
  MSA311_BANDWIDTH_500_HZ = 0b1010,   ///<  500 Hz
} msa311_bandwidth_t;

/** The accelerometer power mode */
typedef enum {
  MSA311_NORMALMODE = 0b00,   ///< Normal (high speed) mode
  MSA311_LOWPOWERMODE = 0b01, ///< Low power (slow speed) mode
  MSA311_SUSPENDMODE = 0b010, ///< Suspend (sleep) mode
} msa311_powermode_t;

/** The accelerometer read resolution */
typedef enum {
  MSA311_RESOLUTION_14 = 0b00, ///< 14-bit resolution
  MSA311_RESOLUTION_12 = 0b01, ///< 12-bit resolution
  MSA311_RESOLUTION_10 = 0b10, ///< 10-bit resolution
  MSA311_RESOLUTION_8 = 0b11,  ///< 8-bit resolution
} msa311_resolution_t;

/** Tap duration parameter */
typedef enum {
  MSA311_TAPDUR_50_MS = 0b000,  ///< 50 millis
  MSA311_TAPDUR_100_MS = 0b001, ///< 100 millis
  MSA311_TAPDUR_150_MS = 0b010, ///< 150 millis
  MSA311_TAPDUR_200_MS = 0b011, ///< 200 millis
  MSA311_TAPDUR_250_MS = 0b100, ///< 250 millis
  MSA311_TAPDUR_375_MS = 0b101, ///< 375 millis
  MSA311_TAPDUR_500_MS = 0b110, ///< 500 millis
  MSA311_TAPDUR_700_MS = 0b111, ///< 50 millis700 millis
} msa311_tapduration_t;

/** Interrupts available */
typedef enum {
  MSA311_INT_ORIENT = 0b100000, ///< Orientation change interrupt
  MSA311_INT_SINGLETAP,         ///< Single tap interrupt
  MSA311_INT_DOUBLETAP,         ///< Double tap interrupt
  MSA311_INT_ACTIVE,            ///< Active motion interrupt
  MSA311_INT_NEWDATA,           ///< New data interrupt
} msa311_interrupt_t;


/** Class for hardware interfacing with an MSA311 accelerometer */
typedef struct Adafruit_MSA311 {
  int16_t x, ///< The last read X acceleration in raw units
      y,     ///< The last read Y acceleration in raw units
      z;     ///< The last read Z acceleration in raw units
  float x_g, ///< The last read X acceleration in 'g'
      y_g,   ///< The last read Y acceleration in 'g'
      z_g;   ///< The last read X acceleration in 'g'

  int32_t _sensorID;
};
void Adafruit_MSA311_init(void);
bool Adafruit_MSA311_begin(uint8_t i2c_addr = MSA311_I2CADDR_DEFAULT, TwoWire *wire = &Wire);

void Adafruit_MSA311_setDataRate(msa311_dataRate_t dataRate);
msa311_dataRate_t Adafruit_MSA311_getDataRate(void);
void Adafruit_MSA311_enableAxes(bool x, bool y, bool z);

void Adafruit_MSA311_setPowerMode(msa311_powermode_t mode);
msa311_powermode_t Adafruit_MSA311_getPowerMode(void);
void Adafruit_MSA311_setBandwidth(msa311_bandwidth_t bandwidth);
msa311_bandwidth_t Adafruit_MSA311_getBandwidth(void);
void Adafruit_MSA311_setRange(msa311_range_t range);
msa311_range_t Adafruit_MSA311_getRange(void);
void Adafruit_MSA311_setResolution(msa311_resolution_t resolution);
msa311_resolution_t Adafruit_MSA311_getResolution(void);

void Adafruit_MSA311_read();

void Adafruit_MSA311_enableInterrupts(bool singletap = false, bool doubletap = false,
                      bool activeX = false, bool activeY = false,
                      bool activeZ = false, bool newData = false,
                      bool freefall = false, bool orient = false);
void Adafruit_MSA311_mapInterruptPin(bool singletap = false, bool doubletap = false,
                      bool activity = false, bool newData = false,
                      bool freefall = false, bool orient = false);

uint8_t Adafruit_MSA311_getClick(void);
uint8_t Adafruit_MSA311_getMotionInterruptStatus(void);
uint8_t Adafruit_MSA311_getDataInterruptStatus(void);

void Adafruit_MSA311_setClick(bool tap_quiet, bool tap_shock,
                msa311_tapduration_t tapduration, uint8_t tapthresh);


#endif
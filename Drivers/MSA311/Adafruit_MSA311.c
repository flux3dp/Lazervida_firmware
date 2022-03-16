/*!
 * @file     Adafruit_MSA311.cpp
 *
 * @mainpage Adafruit MSA311 Accelerometer Breakout
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit MSA311 Accel breakout board
 * ----> https://www.adafruit.com
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Limor Fried (Adafruit Industries)
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_MSA311.h"

static Adafruit_MSA311 msa311;

/**************************************************************************/
/*!
    @brief  Instantiates a new MSA311 class
*/
/**************************************************************************/
void Adafruit_MSA311_init() {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MSA311_begin(I2C_HandleTypeDef *p_hi2c) {
  msa311.p_hi2c = p_hi2c;
  // make sure we're talking to the right chip
  if (MSA311_get_partid() != 0x13) {
    // No MSA301 detected ... return false
    return false;
  }

  // enable all axes
  Adafruit_MSA311_enableAxes(true, true, true);
  // normal mode
  Adafruit_MSA311_setPowerMode(MSA311_NORMALMODE);
  // 500Hz rate
  Adafruit_MSA311_setDataRate(MSA311_DATARATE_500_HZ);
  // 250Hz bw
  Adafruit_MSA311_setBandwidth(MSA311_BANDWIDTH_250_HZ);
  Adafruit_MSA311_setRange(MSA311_RANGE_2_G);

  MSA311_setActiveInterruptThresh(200);
  MSA311_setActiveInterruptDur(MSA311_ACTIVEDUR_4_MS);

  // Enable Active(sudden move) and freefall interrupts
  Adafruit_MSA311_enableInterrupts(false, false, true, true, true, false, true, false);
  Adafruit_MSA311_mapInterruptPin(false, false, true, false, true, false);
  /*
  // DRDY on INT1
  writeRegister8(MSA311_REG_CTRL3, 0x10);
  // Turn on orientation config
  //writeRegister8(MSA311_REG_PL_CFG, 0x40);
  */
  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
  }
  */

  return true;
}

uint8_t MSA311_get_partid() {
  
  uint8_t reg_buf[1] = {MSA311_REG_PARTID};
  uint8_t recv_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, recv_buf, 1, 1000);
  return recv_buf[0];
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the MSA301 (controls power consumption)
    from 1 Hz to 1000Hz
    @param dataRate Enumerated msa311_dataRate_t
*/
/**************************************************************************/
void Adafruit_MSA311_setDataRate(msa311_dataRate_t dataRate) {
  
  uint8_t reg_buf[1] = {MSA311_REG_ODR};
  uint8_t odr_buf[1] = {0};
  uint8_t send_buf[2] = {MSA311_REG_ODR, 0};
  
  // read, modify and write
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, odr_buf, 1, 1000);
  send_buf[1] = (odr_buf[0] & 0b11110000) | dataRate;
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the MSA301 (controls power consumption)
    @return Enumerated msa311_dataRate_t from 1 Hz to 1000Hz
*/
/**************************************************************************/
msa311_dataRate_t Adafruit_MSA311_getDataRate(void) {
  
  uint8_t reg_buf[1] = {MSA311_REG_ODR};
  uint8_t odr_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, odr_buf, 1, 1000);
  return (msa311_dataRate_t)(odr_buf[0] & 0b00001111);
}

/**************************************************************************/
/*!
    @brief  What axes of the accelerometer we want enabled for reading
    @param enableX True to enable X axis
    @param enableY True to enable Y axis
    @param enableZ True to enable Z axis
*/
/**************************************************************************/
void Adafruit_MSA311_enableAxes(bool enableX, bool enableY, bool enableZ) {
  
  uint8_t reg_buf[1] = {MSA311_REG_ODR};
  uint8_t odr_buf[1] = {0};
  uint8_t send_buf[2] = {MSA311_REG_ODR, 0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, odr_buf, 1, 1000);
  send_buf[1] = (odr_buf[0] & 0b00001111) | ((!enableX) << 7) | ((!enableY) << 6) | ((!enableZ) << 5);
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief Set the power mode, MSA311_NORMALMODE, MSA311_LOWPOWERMODE or
    MSA311_SUSPENDMODE
    @param mode Enumerated msa311_powermode_t
*/
/**************************************************************************/
void Adafruit_MSA311_setPowerMode(msa311_powermode_t mode) {
  
  uint8_t reg_buf[1] = {MSA311_REG_POWERMODE};
  uint8_t power_mode_buf[1] = {0};
  uint8_t send_buf[2] = {MSA311_REG_POWERMODE, 0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, power_mode_buf, 1, 1000);
  send_buf[1] = (power_mode_buf[0] & 0b00011111) | (mode << 6);
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief Get the power mode
    @returns Enumerated msa311_powermode_t, MSA311_NORMALMODE,
   MSA311_LOWPOWERMODE or
    MSA311_SUSPENDMODE
*/
/**************************************************************************/
msa311_powermode_t Adafruit_MSA311_getPowerMode(void) {

  uint8_t reg_buf[1] = {MSA311_REG_POWERMODE};
  uint8_t power_mode_buf[1] = {0};

  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, power_mode_buf, 1, 1000);
  return (msa311_powermode_t)((power_mode_buf[0] & 0b11000000) >> 6);
}

/**************************************************************************/
/*!
    @brief Set the bandwidth, ranges from 1.95Hz to 500Hz
    @param bandwidth Enumerated msa311_range_t
*/
/**************************************************************************/
void Adafruit_MSA311_setBandwidth(msa311_bandwidth_t bandwidth) {

  uint8_t reg_buf[1] = {MSA311_REG_POWERMODE};
  uint8_t power_mode_buf[1] = {0};
  uint8_t send_buf[2] = {MSA311_REG_POWERMODE, 0};

  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, power_mode_buf, 1, 1000);
  send_buf[1] = (power_mode_buf[0] & 0b11000000) | (bandwidth << 1);
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief Get the bandwidth
    @return Enumerated msa311_bandwidth_t, ranges from 1.95Hz to 500Hz
*/
/**************************************************************************/
msa311_bandwidth_t Adafruit_MSA311_getBandwidth(void) {
  
  uint8_t reg_buf[1] = {MSA311_REG_POWERMODE};
  uint8_t power_mode_buf[1] = {0};

  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, power_mode_buf, 1, 1000);
  return (msa311_bandwidth_t)((power_mode_buf[0] & 0b00011110) >> 1);
}

/**************************************************************************/
/*!
    @brief Set the resolution range: +-2g, 4g, 8g, or 16g.
    @param range Enumerated msa311_range_t
*/
/**************************************************************************/
void Adafruit_MSA311_setRange(msa311_range_t range) {

  uint8_t reg_buf[1] = {MSA311_REG_RESRANGE};
  uint8_t range_buf[1] = {0};
  uint8_t send_buf[2] = {MSA311_REG_RESRANGE, 0};

  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, range_buf, 1, 1000);
  send_buf[1] = (range_buf[0] & 0b11111100) | (range);
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief Read the resolution range: +-2g, 4g, 8g, or 16g.
    @returns Enumerated msa311_range_t
*/
/**************************************************************************/
msa311_range_t Adafruit_MSA311_getRange(void) {

  uint8_t reg_buf[1] = {MSA311_REG_RESRANGE};
  uint8_t range_buf[1] = {0};

  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, reg_buf, 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, range_buf, 1, 1000);
  return (msa311_range_t)((range_buf[0] & 0b00000011) >> 0);
}

/**************************************************************************/
/*!
  @brief  Read the XYZ data from the accelerometer and store in the internal
  x, y and z (and x_g, y_g, z_g) member variables.
*/
/**************************************************************************/

void Adafruit_MSA311_read(void) {

  int i;
  uint8_t reg_buf[6] = {MSA311_REG_OUT_X_L, MSA311_REG_OUT_X_H, 
                        MSA311_REG_OUT_Y_L, MSA311_REG_OUT_Y_H, 
                        MSA311_REG_OUT_Z_L, MSA311_REG_OUT_Z_H};
  uint8_t acc_buf[6] = {0};

  for (i = 0; i < 6; i++) {
    HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &reg_buf[i], 1, 1000);
    HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &acc_buf[i], 1, 1000);
  }

  msa311.x = acc_buf[0];
  msa311.x |= acc_buf[1] << 8;
  msa311.y = acc_buf[2];
  msa311.y |= acc_buf[3] << 8;
  msa311.z = acc_buf[4];
  msa311.z |= acc_buf[5] << 8;

  // 12 bits of data in 16 bit range
  msa311.x >>= 4;
  msa311.y >>= 4;
  msa311.z >>= 4;

  msa311_range_t range = Adafruit_MSA311_getRange();
  float scale = 1;
  if (range == MSA311_RANGE_16_G)
    scale = 128;
  if (range == MSA311_RANGE_8_G)
    scale = 256;
  if (range == MSA311_RANGE_4_G)
    scale = 512;
  if (range == MSA311_RANGE_2_G)
    scale = 1024;

  msa311.x_g = (float)msa311.x / scale;
  msa311.y_g = (float)msa311.y / scale;
  msa311.z_g = (float)msa311.z / scale;
}

float MSA311_get_x_data(void) {
  return msa311.x_g;
}

float MSA311_get_y_data(void) {
  return msa311.y_g;
}

float MSA311_get_z_data(void) {
  return msa311.z_g;
}


/**************************************************************************/
/*!
  @brief  Set the click detection register thresholds
  @param  tap_quiet TAP_QUIET flag (check datasheet for details)
  @param  tap_shock TAP_SHOCK flag (check datasheet for details)
  @param  tapduration How long to listen for a second tap (check datasheet for
  details)
  @param  tapthresh How strong the tap signal has to be (check datasheet for
  details)
*/
/**************************************************************************/

void Adafruit_MSA311_setClick(bool tap_quiet, bool tap_shock,
                               msa311_tapduration_t tapduration,
                               uint8_t tapthresh) {

  uint8_t send_tap_dur_buf[2] = {MSA311_REG_TAPDUR, 
                                (tap_quiet << 7 | tap_shock << 6 | tapduration << 0)};
  uint8_t send_tap_thresh_buf[2] = {MSA311_REG_TAPTH, 
                                tapthresh};

  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_tap_dur_buf, 2, 1000);
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_tap_thresh_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent click detect status register value
    @returns 8 bits of interrupt status, check datasheet for what CLICKSTATUS
   bits are
*/
/**************************************************************************/
uint8_t Adafruit_MSA311_getClick(void) {

  uint8_t reg_buf[1] = {MSA311_REG_CLICKSTATUS};
  uint8_t tap_status_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &reg_buf[0], 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, tap_status_buf, 1, 1000);
  return tap_status_buf[0];
}

/**************************************************************************/
/*!
    @brief  Set which interrupts are enabled
    @param singletap Whether to trigger INT on single tap interrupt
    @param doubletap Whether to trigger INT on double tap interrupt
    @param activeX Whether to trigger INT on X axis activity interrupt
    @param activeY Whether to trigger INT on Y axis activity interrupt
    @param activeZ Whether to trigger INT on Z axis activity interrupt
    @param newData Whether to trigger INT on new data available interrupt
    @param freefall Whether to trigger INT on freefall interrupt
    @param orient Whether to trigger INT on orientation interrupt
*/
/**************************************************************************/
void Adafruit_MSA311_enableInterrupts(bool singletap, bool doubletap,
                                       bool activeX, bool activeY, bool activeZ,
                                       bool newData, bool freefall,
                                       bool orient) {
  uint8_t send_int_en0_buf[2] = {MSA311_REG_INTSET0, 0};
  uint8_t send_int_en1_buf[2] = {MSA311_REG_INTSET1, 0};

  uint8_t irqs = 0;
  irqs |= orient << 6;
  irqs |= singletap << 5;
  irqs |= doubletap << 4;
  irqs |= activeX << 0;
  irqs |= activeY << 1;
  irqs |= activeZ << 2;
  send_int_en0_buf[1] =irqs;
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_int_en0_buf, 2, 1000);
  irqs = 0;
  irqs |= newData << 4;
  irqs |= freefall << 3;
  send_int_en1_buf[1] =irqs;
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_int_en1_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief  Set which interrupts are mapped to the INT pin
    @param singletap Whether to trigger INT on single tap interrupt
    @param doubletap Whether to trigger INT on double tap interrupt
    @param activity Whether to trigger INT on activity interrupt
    @param newData Whether to trigger INT on new data available interrupt
    @param freefall Whether to trigger INT on freefall interrupt
    @param orient Whether to trigger INT on orientation interrupt
*/
/**************************************************************************/

void Adafruit_MSA311_mapInterruptPin(bool singletap, bool doubletap,
                                      bool activity, bool newData,
                                      bool freefall, bool orient) {
  uint8_t send_int_map0_buf[2] = {MSA311_REG_INTMAP0, 0};
  uint8_t send_int_map1_buf[2] = {MSA311_REG_INTMAP1, 0};
  
  uint8_t int_pin_irq_map = 0;
  int_pin_irq_map |= orient << 6;
  int_pin_irq_map |= singletap << 5;
  int_pin_irq_map |= doubletap << 4;
  int_pin_irq_map |= activity << 2;
  int_pin_irq_map |= freefall << 0;
  send_int_map0_buf[1] = int_pin_irq_map;
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_int_map0_buf, 2, 1000);
  int_pin_irq_map = newData << 0;
  send_int_map1_buf[1] = int_pin_irq_map;
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_int_map1_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief  Gets the threshold value for active interrupt
    @returns 
*/
/**************************************************************************/

uint8_t MSA311_getActiveInterruptThresh() {
  
  uint8_t reg_buf[1] = {MSA311_REG_ACTIVETH};
  uint8_t active_th_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &reg_buf[0], 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, active_th_buf, 1, 1000);
  return active_th_buf[0];
}

/**************************************************************************/
/*!
    @brief  Sets the threshold value for active interrupt
    @param threshold to be set 
                    3.91mg/LSB for 2g range
                    7.81mg/LSB for 4g range
                    15.625mg/LSB for 8g range
                    31.25mg/LSB for 16g range
*/
/**************************************************************************/

void MSA311_setActiveInterruptThresh(uint8_t threshold) {
  
  uint8_t send_buf[2] = {MSA311_REG_ACTIVETH, threshold};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}

/**************************************************************************/
/*!
    @brief  Gets the consecutive duration value for active interrupt
    @returns 
*/
/**************************************************************************/

msa311_activeduration_t MSA311_getActiveInterruptDur() {
  uint8_t reg_buf[1] = {MSA311_REG_ACTIVEDUR};
  uint8_t active_dur_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &reg_buf[0], 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, active_dur_buf, 1, 1000);
  return (msa311_activeduration_t)(active_dur_buf[0]);
}

/**************************************************************************/
/*!
    @brief  Sets the consecutive duration value for active interrupt
    @param duration_ms 
*/
/**************************************************************************/

void MSA311_setActiveInterruptDur(msa311_activeduration_t duration_ms) {

  uint8_t send_buf[2] = {MSA311_REG_ACTIVEDUR, duration_ms};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, send_buf, 2, 1000);
}


/**************************************************************************/
/*!
    @brief  Gets the most recent motion interrupt status register value
    @returns 8 bits of interrupt status, check datasheet for what MOTION bits
   are
*/
/**************************************************************************/

uint8_t Adafruit_MSA311_getMotionInterruptStatus(void) {

  uint8_t reg_buf[1] = {MSA311_REG_MOTIONINT};
  uint8_t motion_int_status_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &reg_buf[0], 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, motion_int_status_buf, 1, 1000);
  return motion_int_status_buf[0];
}

/**************************************************************************/
/*!
    @brief  Gets the most recent data interrupt status register value
    @returns 8 bits of interrupt status, check datasheet for what DATAINT bits
   are
*/
/**************************************************************************/

uint8_t Adafruit_MSA311_getDataInterruptStatus(void) {

  uint8_t reg_buf[1] = {MSA311_REG_DATAINT};
  uint8_t data_int_status_buf[1] = {0};
  
  HAL_I2C_Master_Transmit(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, &reg_buf[0], 1, 1000);
  HAL_I2C_Master_Receive(msa311.p_hi2c, MSA311_I2CADDR_DEFAULT, data_int_status_buf, 1, 1000);
  return data_int_status_buf[0];
}

void MSA311_print_config(void) {
  printString("[MSA311 config:\n");
  printString("Data rate:");
  print_uint32_base2_ndigit(Adafruit_MSA311_getDataRate(), 4);
  printString("\nPower mode:");
  print_uint32_base2_ndigit(Adafruit_MSA311_getPowerMode(), 2);
  printString("\nBandwidth:");
  print_uint32_base2_ndigit(Adafruit_MSA311_getBandwidth(), 4);
  printString("\nRange:");
  print_uint32_base2_ndigit(Adafruit_MSA311_getRange(), 2);
  printString("\n]\n");
}

void MSA311_print_status() {
  printString("[MSA311 status:\n");

  Adafruit_MSA311_read();
  printString("(x, y, z) = (");
  print_uint32_base10(msa311.x);
  printString(", ");
  print_uint32_base10(msa311.y);
  printString(", ");
  print_uint32_base10(msa311.z);
  printString(")\n");
  printString("(x_g, y_g, z_g) = (");
  printFloat(msa311.x_g, 2);
  printString(", ");
  printFloat(msa311.y_g, 2);
  printString(", ");
  printFloat(msa311.z_g, 2);
  printString(")\n");
  printString("Click status:");
  print_uint8_base2_ndigit(Adafruit_MSA311_getClick(), 8);
  printString("\nMotion INT status:");
  print_uint8_base2_ndigit(Adafruit_MSA311_getMotionInterruptStatus(), 8);
  printString("\nData INT status:");
  print_uint8_base2_ndigit(Adafruit_MSA311_getDataInterruptStatus(), 8);
  printString("\n]\n");
}

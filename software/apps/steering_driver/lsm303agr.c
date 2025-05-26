// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>

#include "lsm303agr.h"
#include "nrf_delay.h"

// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t* i2c_manager = NULL;

// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
static uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ (i2c_addr, &rx_buf, 1, 0)
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  if (result != NRF_SUCCESS) {
    // Likely error codes:
    //  NRF_ERROR_INTERNAL            (0x0003) - something is wrong with the driver itself
    //  NRF_ERROR_INVALID_ADDR        (0x0010) - buffer passed was in Flash instead of RAM
    //  NRF_ERROR_BUSY                (0x0011) - driver was busy with another transfer still
    //  NRF_ERROR_DRV_TWI_ERR_OVERRUN (0x8200) - data was overwritten during the transaction
    //  NRF_ERROR_DRV_TWI_ERR_ANACK   (0x8201) - i2c device did not acknowledge its address
    //  NRF_ERROR_DRV_TWI_ERR_DNACK   (0x8202) - i2c device did not acknowledge a data byte
    printf("I2C transaction failed! Error: %lX\n", result);
  }

  return rx_buf;
}

// Helper function to perform a 1-byte I2C write of a given register
//
// i2c_addr - address of the device to write to
// reg_addr - address of the register within the device to write
static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  //TODO: implement me
  //Note: there should only be a single two-byte transfer to be performed
  uint8_t tx_buf[2] = {reg_addr, data};
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &tx_buf, sizeof(tx_buf), 0)
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  }
}

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c) {
  i2c_manager = i2c;

  // ---Initialize Accelerometer---

  // Reboot acclerometer
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG5_A, 0x80);
  nrf_delay_ms(100); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG4_A, 0x80);

  // Configure accelerometer at 100Hz, normal mode (10-bit)
  // Enable x, y and z axes
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG1_A, 0x57);

  // Read WHO AM I register
  // Always returns the same value if working
  //TODO: read the Accelerometer WHO AM I register and check the result
  uint8_t whoamiA = i2c_reg_read(LSM303AGR_ACC_ADDRESS, WHO_AM_I_A);
  printf("ACC WHOAMI = 0x%02X\n", whoamiA);

  // ---Initialize Magnetometer---

  // Reboot magnetometer
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, 0x40);
  nrf_delay_ms(100); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_C_M, 0x10);

  // Configure magnetometer at 100Hz, continuous mode
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, 0x0C);

  // Read WHO AM I register
  //TODO: read the Magnetometer WHO AM I register and check the result
  uint8_t whoamiM = i2c_reg_read(LSM303AGR_MAG_ADDRESS, WHO_AM_I_M);
  printf("MAG WHOAMI = 0x%02X\n", whoamiM);

  // ---Initialize Temperature---

  // Enable temperature sensor
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, TEMP_CFG_REG_A, 0xC0);
}

// Read the internal temperature sensor
//
// Return measurement as floating point value in degrees C
float lsm303agr_read_temperature(void) {
  //TODO: implement me
  int8_t  lsb = (int8_t)i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_TEMP_L_A);
  int8_t  msb = (int8_t)i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_TEMP_H_A);
  
  int16_t raw = ( (int16_t)msb << 8 ) | (uint8_t)lsb;

  float temp_c = ((float)raw * (1.0f / 256.0f)) + 25.0f;
  return temp_c;
}

// FS = ±2 g, normal mode → 3.9 mg/LSB (typical) → 0.0039 g/LSB
#define ACC_SENSITIVITY_G_PER_LSB  (0.0039f)

lsm303agr_measurement_t lsm303agr_read_accelerometer(void) {
  //TODO: implement me
  lsm303agr_measurement_t m = {0};

  uint8_t xl = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_L_A);
  uint8_t xh = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_H_A);
  int16_t x_raw = ((int16_t)xh << 8) | xl;
  x_raw >>= 6;
  m.x_axis = (float)x_raw * ACC_SENSITIVITY_G_PER_LSB;

  uint8_t yl = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_L_A);
  uint8_t yh = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_H_A);
  int16_t y_raw = ((int16_t)yh << 8) | yl;
  y_raw >>= 6;
  m.y_axis = (float)y_raw * ACC_SENSITIVITY_G_PER_LSB;

  uint8_t zl = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_L_A);
  uint8_t zh = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_H_A);
  int16_t z_raw = ((int16_t)zh << 8) | zl;
  z_raw >>= 6;
  m.z_axis = (float)z_raw * ACC_SENSITIVITY_G_PER_LSB;

  return m;
}

// M_So = 1.5 mgauss/LSB → 0.1 µT per mgauss → 0.15 µT/LSB
#define MAG_SENSITIVITY_UT_PER_LSB (1.5f * 0.1f)

lsm303agr_measurement_t lsm303agr_read_magnetometer(void) {
  //TODO: implement me
  lsm303agr_measurement_t m = {0};

  uint8_t xl = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTX_L_REG_M);
  uint8_t xh = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTX_H_REG_M);
  int16_t x_raw = ((int16_t)xh << 8) | xl;
  m.x_axis = (float)x_raw * MAG_SENSITIVITY_UT_PER_LSB;

  uint8_t yl = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTY_L_REG_M);
  uint8_t yh = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTY_H_REG_M);
  int16_t y_raw = ((int16_t)yh << 8) | yl;
  m.y_axis = (float)y_raw * MAG_SENSITIVITY_UT_PER_LSB;

  uint8_t zl = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTZ_L_REG_M);
  uint8_t zh = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTZ_H_REG_M);
  int16_t z_raw = ((int16_t)zh << 8) | zl;
  m.z_axis = (float)z_raw * MAG_SENSITIVITY_UT_PER_LSB;

  return m;
}


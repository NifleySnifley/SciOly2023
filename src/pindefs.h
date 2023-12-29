#ifndef PINDEFS_H
#define PINDEFS_H


#ifdef PICO_FLASH_SIZE_BYTES
#include "hardware/i2c.h"
#endif
#include "utils.h"
#include <array>

#ifdef PICO_FLASH_SIZE_BYTES

#define LED_R 21
#define LED_Y 20
#define LED_G 19
#define LED_B 18

#define START_BTN 26

#define MOT_IA1 2
#define MOT_IA2 3
#define MOT_IB1 4
#define MOT_IB2 5

#define ENC_A_A 10
#define ENC_A_B 11
#define ENC_B_A 12
#define ENC_B_B 13

#define IMU_SDA 14
#define IMU_SCL 15
#define IMU_I2C i2c1

#define LIDAR_SDA 8
#define LIDAR_SCL 9
#define LIDAR_I2C i2c0
// white
#define LIDAR_0_INT 6
#define LIDAR_0_CE 16
// green
#define LIDAR_1_INT 7
#define LIDAR_1_CE 17
#endif

#endif
#include "IMU.h"
#include <Wire.h>
#include "config.h"
#include "oled.h"

bool is_inited_imu = true;
ICM_20948_I2C *imu = nullptr;


#define Y_HIGH 20
#define Y_PIXEL 10
#define AD0_VAL 0

ICM_20948_I2C *getIMU(void)
{
    return imu;
}

void imu_init(void)
{
    imu = new ICM_20948_I2C();

    //imu->begin(Wire, ICM20948_ADDR, ICM20948_INT_PIN);
    //imu->begin(Wire, ICM20948_ADDR, ICM20948_INT_PIN);
    imu->begin();
    imu->enableDebugging();    
    if (imu->status != ICM_20948_Stat_Ok)
    {
        Serial.println("setup imu sensor FAIL");
        is_inited_imu = false;
        return;
    }
    Serial.println("ICM_20948_Stat_Ok");
}

void imu_loop()
{
    static uint32_t Millis;

    if (imu->dataReady())
    {
        imu->getAGMT();

        uint8_t x_h = constrain(abs(imu->accX()) / 150, 0, Y_PIXEL);
        Serial.printf("imu->accX() : %f\n", imu->accX());
        // y
        uint8_t y_h = constrain(abs(imu->accY()) / 150, 0, Y_PIXEL);
        // z
        uint8_t z_h = constrain(abs(imu->accZ()) / 150, 0, Y_PIXEL);
    }
}

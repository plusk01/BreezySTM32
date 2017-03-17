/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "atomic.h"
// #include "build/build_config.h"
// #include "build/debug.h"

// #include "common/maths.h"
// #include "common/utils.h"

#include "nvic.h"

#include "system.h"
#include "io.h"
#include "exti.h"
// #include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6500.h"
#include "accgyro_mpu.h"


mpuResetFnPtr mpuResetFn;

// #ifndef MPU_I2C_INSTANCE
// #define MPU_I2C_INSTANCE I2C_DEVICE
// #endif

// #define MPU_ADDRESS             0x68

// // WHO_AM_I register contents for MPU3050, 6050 and 6500
// #define MPU6500_WHO_AM_I_CONST              (0x70)
// #define MPUx0x0_WHO_AM_I_CONST              (0x68)

// #define MPU_INQUIRY_MASK   0x7E

/*
 * Gyro interrupt service routine
 */
// #if defined(MPU_INT_EXTI)
static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
// #ifdef DEBUG_MPU_DATA_READY_INTERRUPT
//     static uint32_t lastCalledAtUs = 0;
//     const uint32_t nowUs = micros();
//     debug[0] = (uint16_t)(nowUs - lastCalledAtUs);
//     lastCalledAtUs = nowUs;
// #endif
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
    if (gyro->update) {
        gyro->update(gyro);
    }
// #ifdef DEBUG_MPU_DATA_READY_INTERRUPT
//     const uint32_t now2Us = micros();
//     debug[1] = (uint16_t)(now2Us - nowUs);
// #endif
}
// #endif

static void mpuIntExtiInit(gyroDev_t *gyro)
{
// #if defined(MPU_INT_EXTI)
    if (!gyro->mpuIntExtiConfig) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiConfig->tag);

// #ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
// #endif

    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);   // TODO - maybe pullup / pulldown ?

    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(mpuIntIO, true);

// #else
//     UNUSED(gyro);
// #endif
}

bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    bool ack = acc->mpuConfiguration.readFn(MPU_RA_ACCEL_XOUT_H, 6, data);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

void mpuGyroSetIsrUpdate(gyroDev_t *gyro, sensorGyroUpdateFuncPtr updateFn)
{
    ATOMIC_BLOCK(NVIC_PRIO_MPU_INT_EXTI) {
        gyro->update = updateFn;
    }
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = gyro->mpuConfiguration.readFn(gyro->mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuCheckDataReady(gyroDev_t* gyro)
{
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady= false;
    } else {
        ret = false;
    }
    return ret;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro)
{
    uint8_t mpu6500Sensor = mpu6500SpiDetect();
    if (mpu6500Sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = mpu6500Sensor;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.readFn = mpu6500ReadRegister;
        gyro->mpuConfiguration.writeFn = mpu6500WriteRegister;
        return true;
    }

    UNUSED(gyro);
    return false;
}
#endif

mpuDetectionResult_t *mpuDetect(gyroDev_t *gyro)
{

    // MPU datasheet specifies 30ms.
    delay(35);


    bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult(gyro);
    UNUSED(detectedSpiSensor);

    return &gyro->mpuDetectionResult;
}

void mpuGyroInit(gyroDev_t *gyro)
{
    mpuIntExtiInit(gyro);
}

/*
   drv_ms5611.c : driver for Measurement Specialties MS5611 barometer

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_ms5611.c

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <breezystm32.h>
#include <limits.h>

// MS5611, Standard address 0x77
#define MS5611_ADDR             0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

static uint32_t ms5611_ut;  // static result of temperature measurement
static uint32_t ms5611_up;  // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t  ms5611_osr = CMD_ADC_4096;
static float altitude = 0.0;
static bool calibrated = false;

static const int16_t max_pressure = 164868;
static const int16_t min_pressure = 222;
static const int16_t dx = 329;
static const int16_t num_entries = 500;
static const int16_t lookup_table[500] = {
  30480,	27872,	26342,	25223,	24329,	23579,	22929,	22354,	21837,	21365,
  20932,	20530,	20156,	19804,	19473,	19160,	18862,	18579,	18308,	18049,
  17800,	17561,	17330,	17108,	16893,	16685,	16483,	16288,	16098,	15913,
  15734,	15559,	15388,	15222,	15060,	14901,	14746,	14594,	14446,	14300,
  14158,	14018,	13881,	13747,	13615,	13486,	13358,	13233,	13110,	12989,
  12870,	12753,	12638,	12525,	12413,	12303,	12194,	12087,	11981,	11877,
  11775,	11673,	11573,	11475,	11377,	11281,	11186,	11092,	10999,	10907,
  10816,	10727,	10638,	10551,	10464,	10378,	10293,	10209,	10126,	10044,
  9963,		9882,	9802,	9723,	9645,	9567,	9491,	9415,	9339,	9265,
  9190,		9117,	9044,	8972,	8901,	8830,	8760,	8690,	8621,	8553,
  8485,		8417,	8350,	8284,	8218,	8153,	8088,	8023,	7960,	7896,
  7833,		7771,	7709,	7647,	7586,	7525,	7465,	7405,	7346,	7287,
  7228,		7170,	7112,	7054,	6997,	6940,	6884,	6828,	6772,	6717,
  6662,		6607,	6553,	6499,	6446,	6392,	6339,	6286,	6234,	6182,
  6130,		6079,	6028,	5977,	5926,	5876,	5826,	5776,	5726,	5677,
  5628,		5579,	5531,	5483,	5435,	5387,	5340,	5293,	5246,	5199,
  5152,		5106,	5060,	5014,	4969,	4923,	4878,	4833,	4789,	4744,
  4700,		4656,	4612,	4568,	4525,	4482,	4439,	4396,	4353,	4311,
  4269,		4227,	4185,	4143,	4101,	4060,	4019,	3978,	3937,	3897,
  3856,		3816,	3776,	3736,	3696,	3656,	3617,	3578,	3539,	3500,
  3461,		3422,	3384,	3345,	3307,	3269,	3231,	3193,	3156,	3118,
  3081,		3044,	3007,	2970,	2933,	2897,	2860,	2824,	2788,	2752,
  2716,		2680,	2644,	2609,	2573,	2538,	2503,	2468,	2433,	2398,
  2364,		2329,	2295,	2260,	2226,	2192,	2158,	2124,	2091,	2057,
  2024,		1990,	1957,	1924,	1891,	1858,	1825,	1792,	1760,	1727,
  1695,		1662,	1630,	1598,	1566,	1534,	1503,	1471,	1439,	1408,
  1376,		1345,	1314,	1283,	1252,	1221,	1190,	1159,	1129,	1098,
  1068,		1037,	1007,	977,	947,	917,	887,	857,	828,	798,
  768,		739,	709,	680,	651,	622,	593,	564,	535,	506,
  477,		449,	420,	392,	363,	335,	307,	278,	250,	222,
  194,		167,	139,	111,	83,     56,     28,	1,	-27,	-54,
  -81,		-108,	-135,	-162,	-189,	-216,	-243,	-270,	-296,	-323,
  -349,		-376,	-402,	-429,	-455,	-481,	-507,	-533,	-559,	-585,
  -611,		-637,	-663,	-688,	-714,	-740,	-765,	-790,	-816,	-841,
  -866,		-892,	-917,	-942,	-967,	-992,	-1017,	-1042,	-1066,	-1091,
  -1116,	-1140,	-1165,	-1189,	-1214,	-1238,	-1263,	-1287,	-1311,	-1335,
  -1359,	-1384,	-1408,	-1431,	-1455,	-1479,	-1503,	-1527,	-1550,	-1574,
  -1598,	-1621,	-1645,	-1668,	-1692,	-1715,	-1738,	-1761,	-1785,	-1808,
  -1831,	-1854,	-1877,	-1900,	-1923,	-1945,	-1968,	-1991,	-2014,	-2036,
  -2059,	-2082,	-2104,	-2127,	-2149,	-2171,	-2194,	-2216,	-2238,	-2260,
  -2282,	-2305,	-2327,	-2349,	-2371,	-2393,	-2414,	-2436,	-2458,	-2480,
  -2502,	-2523,	-2545,	-2566,	-2588,	-2609,	-2631,	-2652,	-2674,	-2695,
  -2716,	-2738,	-2759,	-2780,	-2801,	-2822,	-2843,	-2864,	-2885,	-2906,
  -2927,	-2948,	-2969,	-2989,	-3010,	-3031,	-3051,	-3072,	-3093,	-3113,
  -3134,	-3154,	-3175,	-3195,	-3215,	-3236,	-3256,	-3276,	-3296,	-3317,
  -3337,	-3357,	-3377,	-3397,	-3417,	-3437,	-3457,	-3477,	-3496,	-3516,
  -3536,	-3556,	-3575,	-3595,	-3615,	-3634,	-3654,	-3673,	-3693,	-3712,
  -3732,	-3751,	-3771,	-3790,	-3809,	-3829,	-3848,	-3867,	-3886,	-3905,
  -3924,	-3944,	-3963,	-3982,	-4001,	-4020,	-4038,	-4057,	-4076,	-4095,
  -4114,	-4133,	-4151,	-4170,	-4189,	-4207,	-4226,	-4244,	-4263,	-4282,
};

static float fast_alt(float pressure)
{

  if(pressure < max_pressure && pressure > min_pressure)
  {
    float t = (float)num_entries*(pressure - (float)min_pressure) / (float)(max_pressure - min_pressure);
    int16_t index = (uint16_t)t;
    float dp = t - (float)index;

    float out = 0.0;

    if (index < num_entries - 1)
    {
      out = lookup_table[index]/10.0 + dp * (lookup_table[index + 1] - lookup_table[index])/10.0;
    }
    else
    {
      out = lookup_table[index]/10.0 + dp * (lookup_table[index] - lookup_table[index - 1])/10.0;
    }

    return out;
  }
  else
    return 0.0;
}

static void ms5611_reset(void)
{
  i2cWrite(MS5611_ADDR, CMD_RESET, 1);
  delayMicroseconds(2800);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
  uint8_t rxbuf[2] = { 0, 0 };
  i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
  return rxbuf[0] << 8 | rxbuf[1];
}

int8_t ms5611_crc(uint16_t *prom)
{
  int32_t i, j;
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;

  bool blankEeprom = true;

  for (i = 0; i < 16; i++) {
    if (prom[i >> 1]) {
      blankEeprom = false;
    }
    if (i & 1)
      res ^= ((prom[i >> 1]) & 0x00FF);
    else
      res ^= (prom[i >> 1] >> 8);
    for (j = 8; j > 0; j--) {
      if (res & 0x8000)
        res ^= 0x1800;
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (!blankEeprom && crc == ((res >> 12) & 0xF))
    return 0;

  return -1;
}

static uint32_t ms5611_read_adc(void)
{
  uint8_t rxbuf[3];
  i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
  return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(void)
{
  i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(void)
{
  ms5611_ut = ms5611_read_adc();
}

static void ms5611_start_up(void)
{
  i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(void)
{
  ms5611_up = ms5611_read_adc();
}

static void ms5611_calculate(uint32_t *pressure, uint32_t *temperature)
{
  uint32_t press;
  int64_t temp;
  int64_t delt;
  int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
  int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
  int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
  temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

  if (temp < 2000) { // temperature lower than 20degC
    delt = temp - 2000;
    delt = 5 * delt * delt;
    off -= delt >> 1;
    sens -= delt >> 2;
    if (temp < -1500) { // temperature lower than -15degC
      delt = temp + 1500;
      delt = delt * delt;
      off -= 7 * delt;
      sens -= (11 * delt) >> 1;
    }
    temp -= ((dT * dT) >> 31);
  }
  press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;


  if (pressure)
    *pressure = press;
  if (temperature)
    *temperature = temp;
}

typedef void (*baroOpFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(uint32_t *pressure, uint32_t *temperature);             // baro calculation (filled params are pressure and temperature)

typedef struct baro_t {
  uint16_t ut_delay;
  uint16_t up_delay;
  baroOpFuncPtr start_ut;
  baroOpFuncPtr get_ut;
  baroOpFuncPtr start_up;
  baroOpFuncPtr get_up;
  baroCalculateFuncPtr calculate;
} baro_t;

static baro_t baro;

static uint32_t baroTemperature;
static uint32_t baroPressure;

// =======================================================================================


bool ms5611_init(void)
{
  bool ack = false;
  uint8_t sig;
  int i;

  //    gpio_config_t gpio;
  //    gpio.pin = Pin_13;
  //    gpio.speed = Speed_2MHz;
  //    gpio.mode = Mode_Out_PP;
  //    gpioInit(GPIOC, &gpio);

  delay(10); // No idea how long the chip takes to power-up, but let's make it 10ms

  ack = i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
  if (!ack)
    return false;

  ms5611_reset();

  // read all coefficients
  for (i = 0; i < PROM_NB; i++)
    ms5611_c[i] = ms5611_prom(i);
  // check crc, bail out if wrong
  if (ms5611_crc(ms5611_c) != 0)
    return false;

  // TODO prom + CRC
  baro.ut_delay = 10000;
  baro.up_delay = 10000;
  baro.start_ut = ms5611_start_ut;
  baro.get_ut = ms5611_get_ut;
  baro.start_up = ms5611_start_up;
  baro.get_up = ms5611_get_up;
  baro.calculate = ms5611_calculate;

  calibrated = false;

  return true;
}

void ms5611_update(void)
{
  static uint64_t baroDeadline = 0;
  static int state = 0;

  uint64_t currentTime = micros();

  if ((int64_t)(currentTime - baroDeadline) < 0)
    return;

  baroDeadline = currentTime;

  if (state) {
    baro.get_up();
    baro.start_ut();
    baroDeadline += baro.ut_delay;
    baro.calculate(&baroPressure, &baroTemperature);
    state = 0;
  } else {
    baro.get_ut();
    baro.start_up();
    state = 1;
    baroDeadline += baro.up_delay;
    baro.calculate(&baroPressure, &baroTemperature);
  }
}

/*=======================================================
 * Asynchronous I2C Read Functions:
 * These methods use the asynchronous I2C
 * read capability on the naze32.
 */

static uint8_t pressure_buffer[3];
static uint8_t temp_buffer[3];

static uint8_t temp_command = 1;
static uint8_t pressure_command = 1;
static volatile uint8_t temp_start_status = 0;
static volatile uint8_t temp_read_status = 0;
static volatile uint8_t pressure_read_status = 0;
static volatile uint8_t pressure_start_status = 0;
static uint8_t baro_state = 0;
static volatile uint64_t next_update_us = 0;

void temp_request_CB(void)
{
  next_update_us = micros() + 10000;
  baro_state = 0;
}

void pressure_request_CB(void)
{
  next_update_us = micros() + 10000;
  baro_state = 1;

}

void pressure_read_CB(void)
{
  uint32_t read = (pressure_buffer[0] << 16) | (pressure_buffer[1] << 8) | pressure_buffer[2];
  if(read != 0)
  {
    ms5611_up = read;
    baro.calculate(&baroPressure, &baroTemperature);
  }

  // start a temperature update
  i2c_queue_job(WRITE,
                MS5611_ADDR,
                CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr,
                &temp_command,
                1,
                &temp_start_status,
                &temp_request_CB);
}

static void temp_read_CB(void)
{
  uint32_t read = (temp_buffer[0] << 16) | (temp_buffer[1] << 8) | temp_buffer[2];
  if(read != 0)
  {
    ms5611_ut = read;
    baro.calculate(&baroPressure, &baroTemperature);
  }

  // start a pressure read
  i2c_queue_job(WRITE,
                MS5611_ADDR,
                CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr,
                &pressure_command,
                1,
                &pressure_start_status,
                &pressure_request_CB);
}


void ms5611_request_async_update(void)
{
  // if it's not time to do anything, just return
  if (next_update_us > micros())
  {
    return;
  }

  else if(baro_state == 1)
  {
    // Read The pressure started earlier
    i2c_queue_job(READ,
                  MS5611_ADDR,
                  CMD_ADC_READ,
                  pressure_buffer,
                  3,
                  &pressure_read_status,
                  &pressure_read_CB);

    // put into a waiting state until the I2C is done
    next_update_us = micros() + 3*baro.up_delay;
  }
  else if(baro_state == 0)
  {
    // Read the temperature started earlier
    i2c_queue_job(READ,
                  MS5611_ADDR,
                  CMD_ADC_READ,
                  temp_buffer,
                  3,
                  &temp_read_status,
                  &temp_read_CB);

    // put into a waiting state until the I2C is done
    next_update_us = micros() + 3*baro.ut_delay;
  }
}

uint32_t ms5611_read_pressure(void)
{
  return baroPressure;
  //  return ms5611_up;
}

uint32_t ms5611_read_temperature(void)
{
  return baroTemperature;
  //  return ms5611_ut;
}

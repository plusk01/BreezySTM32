#include <breezystm32.h>


void setup()
{
  pwmInit(true, false, false, 490, 1000);
}


void loop()
{
  pwmRead(2);

  float distance = sonarRead(0);
  printf("sonar_read = %d.%dm\n", (int32_t)distance, (int32_t)(distance*1000)%1000);
}

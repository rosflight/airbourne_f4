#include "revo_f4.h"

#include "drv_spi.h"
#include "mpu6000.h"
#include "drv_led.h"
#include "vcp.h"
#include "printf.h"

VCP* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}

int main() {

  systemInit();

  VCP vcp;
  vcp.init();
  uartPtr = &vcp;

  init_printf(NULL, _putc);

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  SPI mpu_spi;
  mpu_spi.init(MPU6000_SPI);

  MPU6000 imu;
  imu.init(&mpu_spi);
  float temp;
  float acc[3];
  float gyro[3];
  uint64_t time_us;
  while(1)
  {
    info.toggle();
    imu.read(acc, gyro, &temp, &time_us);
    if (acc[0] == 0xFFFF || acc[0] == 0x0000)
    {
      warn.on();
      printf("error\n");
    }
    else
    {
      warn.off();
      printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
             (int32_t) (acc[0]*1000.0),
             (int32_t) (acc[1]*1000.0),
             (int32_t) (acc[2]*1000.0),
             (int32_t) (gyro[0]*1000.0),
             (int32_t) (gyro[1]*1000.0),
             (int32_t) (gyro[2]*1000.0),
             (int32_t) (temp*1000.0),
             time_us);
    }
    delay(10);
  }
}

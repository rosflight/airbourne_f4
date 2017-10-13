#include <string>
#include "revo_f4.h"

#include "drv_spi.h"
#include "mpu6000.h"
#include "drv_led.h"
#include "vcp.h"
#include "printf.h"
#include "eeprom.h"

VCP* vcpPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    vcpPtr->put_byte(c);
}

int main() {

  systemInit();

  struct ExampleStruct{
    int version;
    std::string name;
    float someOtherValue;
  };
  ExampleStruct data;
  /*
  //To write, uncomment this
  data.version=1;
  data.someOtherValue=3.14154;
  memory_write((&data),sizeof(data));
  //*/
  //*
  //to read, uncomment this
  memory_read(&data,sizeof(data));
  //*/
  printf("version: %d",data.version);
  printf("someOtherValue: %f",data.someOtherValue);
}

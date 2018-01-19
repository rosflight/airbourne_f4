#include <string>
#include "revo_f4.h"

#include "spi.h"
#include "mpu6000.h"
#include "led.h"
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

  VCP vcp;
  vcp.init();
  vcpPtr = &vcp;

  struct ExampleStruct{
    int version;
    std::string name;
    float someOtherValue;
  };
  ExampleStruct data;

  //To write, uncomment this
  data.version=1;
  data.someOtherValue=3.14154f;
  memory_write((&data),sizeof(data));
  
  //to read, uncomment this
  memory_read(&data,sizeof(data));


  printf("version: %d",data.version);
  printf("someOtherValue: %f",data.someOtherValue);
}

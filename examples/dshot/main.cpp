#include "system.h"
#include "led.h"
#include "vcp.h"
#include "printf.h"
#include "revo_f4.h"
#include "dshot.h"

VCP *uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}

int main()
{

    systemInit();

    // logging
    VCP vcp;
    vcp.init();
    uartPtr = &vcp;
    init_printf(NULL, _putc);

    // leds
    LED warn;
    warn.init(LED1_GPIO, LED1_PIN);
    LED info;
    info.init(LED2_GPIO, LED2_PIN);

    delay(1000);

    info.on();

    DSHOT_OUT dshot;

    float test_value = 0;
    uint32_t last_print_ms = 0;
    uint16_t result, packet;

    dshot.init(0);

    while (1)
    {
        if (millis() > last_print_ms + 750)
        {
            // cycle through all output values (0-1) and .05 steps
            test_value += 0.05;
            if (test_value > 1.01) {
                test_value = 0.0;
            }
            
            info.toggle(); // cause why not

            dshot.write(test_value);

            last_print_ms = millis();
        //     printf("Command 0.%02d @ %d Hz\n", (uint32_t)(test_value*100), dshot.dshot_freq_hz);
        }
    }
}

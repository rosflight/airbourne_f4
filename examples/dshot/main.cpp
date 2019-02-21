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

    delay(500);

    info.on();

    DSHOT_OUT dshot;

    dshot.init();

    float test_value = 0;
    uint32_t last_print_ms = 0;
    uint16_t result, packet;

    float ns_per_cyc = 0.0;

    while (1)
    {
        if (millis() > last_print_ms + 50)
        {
            info.toggle(); // cause why not

            // cycle through all output values (0-1) and .05 steps
            test_value += 0.05;
            if (test_value > 1.01) {
                test_value = 0.0;
            }

            result = dshot.write(test_value);
            ns_per_cyc = dshot.getNSCyc();

            last_print_ms = millis();
            printf("For input: 0.%02d -> %04x\n", (uint32_t)(test_value*100), result);
            printf("tim cycles per ns:: %f\n", ns_per_cyc);
        }
    }
}

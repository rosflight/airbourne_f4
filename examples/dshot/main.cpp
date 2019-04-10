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

    // dshot.init(150);
    // dshot.init2(150000);

    float test_value = 0;
    uint32_t last_print_ms = 0;
    uint16_t result, packet;
    printf("startup");
    int i = 0;
    while (i < 10) {
        if (millis() > last_print_ms + 50)
        {
            printf("..");
            last_print_ms = millis();
            i++;
        }
    }

    printf("\n");
    dshot.kflyInit();
    bool toggle = false;

    while (1)
    {
        if (millis() > last_print_ms + 750)
        {
            // cycle through all output values (0-1) and .05 steps
            test_value += 0.05;
            if (test_value > 1.01) {
                test_value = 0.0;
            }
            
            // printf("INIT1!!::\n");
            // dshot.init(1200);

            // printf("INIT2!!::\n");
            // dshot.init2(150000);

            // printf("test write 2\n");
            // dshot.write2(test_value);
            info.toggle(); // cause why not

            // if (toggle) {
                dshot.kflyWrite(0.0);
            // } else {
                // dshot.write2(0.0);
            // }

            toggle = !toggle;

            last_print_ms = millis();
        //     printf("Command 0.%02d @ %d Hz\n", (uint32_t)(test_value*100), dshot.dshot_freq_hz);
        }
    }
}

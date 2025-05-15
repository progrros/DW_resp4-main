#include <port.h>
#include "deca_device_api.h"

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    Sleep(time_ms);
}


/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_usleep(unsigned long time_us)
{
    usleep(time_us);
}

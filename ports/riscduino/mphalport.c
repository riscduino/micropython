#include <unistd.h>
#include "py/mpconfig.h"
#include "debug.h"

void DeviceInit(void) {
    Delay_Init();
    USART_Printf_Init(115200);
}

// Receive single character, blocking until one is available.
int mp_hal_stdin_rx_chr(void) {
    return USART_ReceiveData(UART0);
}

// Send the string of given length.
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
     for(int i=0; i < len; i++)
        USART_SendData(UART0, str[i]);
}

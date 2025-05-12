## UART Initialization

In `sim7080g_uart_init`, the order of parameters to `uart_set_pin` was updated to correctly reflect the signature:

```c
err = uart_set_pin((uart_port_t)sim7080g_uart_config.port_num,
                     sim7080g_uart_config.gpio_num_tx, // TXD pin
                     sim7080g_uart_config.gpio_num_rx, // RXD pin
                     UART_PIN_NO_CHANGE,             // RTS
                     UART_PIN_NO_CHANGE);            // CTS
```

Before, the TXD and RXD pins were reversed.

## C++ Compatibility

An `extern "C"` block was added to the header file to ensure C++ compatibility.

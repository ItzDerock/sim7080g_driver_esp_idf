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

## Time Sync

Ability to sync NTP with `AT+CCLK` command was added. This is performed automatically if needed for MQTTS.

```c
// Get the current timestamp
time_t current_epoch;
ESP_RETURN_ON_ERROR(sim7080g_get_epoch_time_utc(handle, &current_epoch), TAG,
                    "Failed to get current epoch time");
```

## MQTTS

A `use_tls` flag was added to support MQTTS. Pass in a certificate in PEM format for the modem to check against. Example,

```c
sim7080g_mqtt_config_t mqtt_config = {.broker_url = MQTT_BROKER_URL,
                                      .username = MQTT_USERNAME,
                                      .client_id = MQTT_CLIENT_ID,
                                      .client_password = MQTT_CLIENT_PASSWORD,
                                      .port = MQTT_PORT,
                                      .use_tls = USE_MQTTS,
                                      .ssl_context_index = 0,
                                      .ca_cert_filename_on_modem =
                                          "mqtts_ca.pem"};

// ...

ESP_LOGI(TAG, "Initializing modem handle...");
ESP_RETURN_ON_ERROR(sim7080g_init(sim7080g_handle), TAG,
                    "Failed to initialize modem handle");

ESP_LOGI(TAG, "Configuring SSL for MQTTS...");
ESP_RETURN_ON_ERROR(sim7080g_mqtts_configure_ssl(handle, CA_PEM), TAG,
                    "Failed to configure MQTTS SSL.");
```

## Speed

Speed of the module was greatly improved. In the original implementation, the module would always wait the full 5 second timeout before processing a response. The modem might have fully sent a respones, but the library would not process it until the timeout expired. Now, the module will process by chunks and exit once it finds a terminator (i.e. `\r\nOK\r\n`, `OK\r\n`, `\r\nERROR\r\n`, `+CME ERROR:`, or `+CMS ERROR:`).

## Network Polling

Instead of repeadily running CFUN cycles (radio reset) if not registered to a network, instead, poll to see if we can registrate for a full minute (2 second interval, 30 attempts) to see if just need some time to register. If it fails, the go back and CFUN cycle.

#pragma once

#include <esp_err.h>
#include <stdbool.h>

#define SIM7080G_UART_BAUD_RATE 115200
#define SIM87080G_UART_BUFF_SIZE 1024

// TODO - Check these values against  MQTT v3 protocol specs
#define MQTT_BROKER_URL_MAX_CHARS 128
#define MQTT_BROKER_USERNAME_MAX_CHARS 32
#define MQTT_BROKER_CLIENT_ID_MAX_CHARS 32
#define MQTT_BROKER_PASSWORD_MAX_CHARS 32

/// @brief UART config struct defined by user of driver and passed to driver
/// init
/// @note TX and RX here are in the perspective of the SIM7080G, and thus they
/// are swapped in the perspecive of the ESP32
typedef struct {
  int gpio_num_tx;
  int gpio_num_rx;
  int port_num; // This was chosen to be an INT because the esp-idf uart driver
                // takes an int type
} sim7080g_uart_config_t;

/// @brief For config device with MQTT broker.
/// @note Thinger refers to the client password as the 'device credentials'
typedef struct {
  char broker_url[MQTT_BROKER_URL_MAX_CHARS];
  char username[MQTT_BROKER_USERNAME_MAX_CHARS];
  char client_id[MQTT_BROKER_CLIENT_ID_MAX_CHARS];
  char client_password[MQTT_BROKER_PASSWORD_MAX_CHARS];
  uint16_t port;
} sim7080g_mqtt_config_t;

/**
 * @brief MQTT broker connection status values
 */
typedef enum {
  MQTT_STATUS_DISCONNECTED = 0,      // Not connected to broker
  MQTT_STATUS_CONNECTED = 1,         // Connected to broker
  MQTT_STATUS_CONNECTED_SESSION = 2, // Connected with session present
  MQTT_STATUS_MAX = 3
} sim7080g_mqtt_connection_status_t;

typedef enum {
  MQTT_ERR_NONE = 0,
  MQTT_ERR_NETWORK = 1,     // Network error
  MQTT_ERR_PROTOCOL = 2,    // Protocol error
  MQTT_ERR_UNAVAILABLE = 3, // Server unavailable
  MQTT_ERR_TIMEOUT = 4,     // Connection timeout
  MQTT_ERR_REJECTED = 5,    // Connection rejected
  MQTT_ERR_UNKNOWN = 99     // Unknown error
} mqtt_error_code_t;

/**
 * @brief Structure to hold all MQTT parameters
 * @note  DO NOT INCLUDE the 'mqtt://' protocol prefix OR port number in the URL
 * @note  NOTE used for config as of now (just for runtime value checking and
 * logging) - the user only sets the config struct
 */
typedef struct {
  char broker_url[MQTT_BROKER_URL_MAX_CHARS]; // NOTE: DO NOT INCLUDE the
                                              // 'mqtt://' protocol prefix OR
                                              // port num in the URL
  uint16_t port;
  char client_id[MQTT_BROKER_CLIENT_ID_MAX_CHARS];
  char username[MQTT_BROKER_USERNAME_MAX_CHARS];
  char client_password[MQTT_BROKER_PASSWORD_MAX_CHARS];
  uint16_t keepalive;
  bool clean_session;
  uint8_t qos;
  bool retain;
  bool sub_hex;
  bool async_mode;
} mqtt_parameters_t;

typedef struct {
  sim7080g_uart_config_t uart_config;
  sim7080g_mqtt_config_t mqtt_config;
  bool uart_initialized;
  bool mqtt_initialized;
} sim7080g_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Creates a device handle that stores the provided configurations
/// @note This must be called before a device can be init
/// @param sim7080g_handle
/// @param sim7080g_uart_config
/// @param sim7080g_mqtt_config
/// @return
esp_err_t sim7080g_config(sim7080g_handle_t *sim7080g_handle,
                          const sim7080g_uart_config_t sim7080g_uart_config,
                          const sim7080g_mqtt_config_t sim7080g_mqtt_config);

/// @brief Use configured UART and MQTT settings to initialize drivers for this
/// device
/// @note This can only be called after a handle is configured
/// @param sim7080g_handle
/// @return
esp_err_t sim7080g_init(sim7080g_handle_t *sim7080g_handle);

esp_err_t sim7080g_deinit(sim7080g_handle_t *sim7080g_handle);

esp_err_t sim7080g_check_sim_status(const sim7080g_handle_t *sim7080g_handle);

esp_err_t
sim7080g_check_signal_quality(const sim7080g_handle_t *sim7080g_handle,
                              int8_t *rssi_out, uint8_t *ber_out);

/// REPLACES WHAT WAS ORIGINALLY CALLED 'CHECK NETWORK CONFIGURATION' fxn
esp_err_t
sim7080g_get_gprs_attach_status(const sim7080g_handle_t *sim7080g_handle,
                                bool *attached_out);

esp_err_t sim7080g_get_operator_info(const sim7080g_handle_t *sim7080g_handle,
                                     int *operator_code, int *operator_format,
                                     char *operator_name,
                                     int operator_name_len);

esp_err_t sim7080g_get_apn(const sim7080g_handle_t *sim7080g_handle, char *apn,
                           int apn_len);

esp_err_t sim7080g_set_apn(const sim7080g_handle_t *sim7080g_handle,
                           const char *apn);

esp_err_t
sim7080g_app_network_activate(const sim7080g_handle_t *sim7080g_handle);

esp_err_t
sim7080g_app_network_deactivate(const sim7080g_handle_t *sim7080g_handle);

esp_err_t sim7080g_cycle_cfun(const sim7080g_handle_t *sim7080g_handle);

esp_err_t
sim7080g_get_app_network_active(const sim7080g_handle_t *sim7080g_handle,
                                int pdpidx, int *status, char *address,
                                int address_len);

///...... Other functions for interacting with and configure device
// TODO - Create a 'SIM' config struct that holds the SIM card APN (for now -
// later we can add more)

/// @brief Send series of AT commands to set the device various network settings
/// to connect to LTE network bearer
/// @param sim7080g_handle
/// @param apn
/// @return
esp_err_t
sim7080g_connect_to_network_bearer(const sim7080g_handle_t *sim7080g_handle,
                                   const char *apn);

/// @brief Send Series of AT commands to set the device MQTT values to match the
/// driver handle config values
/// @param sim7080g_handle
/// @return
esp_err_t
sim7080g_mqtt_set_parameters(const sim7080g_handle_t *sim7080g_handle);

/// @brief Uses a single AT command to get the current MQTT parameters from the
/// device
/// @note THE mqtt_parameters_t struct is used to store the values THIS IS NOT
/// THE SAME AS THE CONFIG STRUCT
/// @param sim7080g_handle
/// @param params_out
/// @return
esp_err_t sim7080g_mqtt_get_parameters(const sim7080g_handle_t *sim7080g_handle,
                                       mqtt_parameters_t *params_out);

esp_err_t
sim7080g_mqtt_connect_to_broker(const sim7080g_handle_t *sim7080g_handle);

static esp_err_t mqtt_query_parameter(const sim7080g_handle_t *sim7080g_handle,
                                      const char *param_name, char *value_out,
                                      size_t value_size, uint16_t *port_out);

esp_err_t sim7080g_mqtt_get_broker_connection_status(
    const sim7080g_handle_t *sim7080g_handle,
    sim7080g_mqtt_connection_status_t *status_out);

esp_err_t sim7080g_mqtt_publish(const sim7080g_handle_t *sim7080g_handle,
                                const char *topic, const char *message,
                                uint8_t qos, bool retain);

esp_err_t
sim7080g_set_verbose_error_reporting(const sim7080g_handle_t *sim7080g_handle);

esp_err_t
sim7080g_is_physical_layer_connected(const sim7080g_handle_t *sim7080g_handle,
                                     bool *connected);
esp_err_t
sim7080g_is_data_link_layer_connected(const sim7080g_handle_t *sim7080g_handle,
                                      bool *connected);
esp_err_t
sim7080g_is_network_layer_connected(const sim7080g_handle_t *sim7080g_handle,
                                    bool *connected);
// esp_err_t sim7080g_is_transport_layer_connected(const sim7080g_handle_t
// *sim7080g_handle, bool *connected);

// TODO - Implement session layer check once SSL is implemented

esp_err_t sim7080g_is_application_layer_connected(
    const sim7080g_handle_t *sim7080g_handle, bool *connected);

/// @brief Test the UART connection by sending a command and checking for a
/// response
bool sim7080g_test_uart_loopback(sim7080g_handle_t *sim7080g_handle);

#ifdef __cplusplus
}
#endif

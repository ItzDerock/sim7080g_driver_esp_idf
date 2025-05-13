#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <time.h>

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

  // TLS Support
  bool use_tls;
  uint8_t ssl_context_index;          // SSL context index (e.g., 0)
  char ca_cert_filename_on_modem[32]; // Filename of CA cert on modem (e.g.,
                                      // "ca.crt")
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

/**
 * @brief Uploads a file (e.g., certificate) to the modem's filesystem.
 *
 * @param handle Pointer to the sim7080g_handle_t.
 * @param fs_index Filesystem index (e.g., 3 for /customer/).
 * @param filename_on_modem The desired filename on the modem.
 * @param file_data Pointer to the null-terminated string data of the file.
 * @param timeout_ms Timeout for file write operation.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sim7080g_fs_upload_text_file(const sim7080g_handle_t *handle,
                                       uint8_t fs_index,
                                       const char *filename_on_modem,
                                       const char *file_data,
                                       uint32_t timeout_ms);

/**
 * @brief Configures SSL context on the modem for MQTTS.
 * Uploads the CA certificate and sets SSL parameters.
 *
 * @param handle Pointer to the sim7080g_handle_t (must have
 * mqtt_config.use_ssl=true, mqtt_config.ssl_context_index, and
 * mqtt_config.ca_cert_filename_on_modem set).
 * @param ca_cert_pem_data Null-terminated string containing the CA certificate
 * in PEM format.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sim7080g_mqtts_configure_ssl(const sim7080g_handle_t *handle,
                                       const char *ca_cert_pem_data);

/**
 * @brief Gets the current UTC time from the modem and returns it as an epoch
 * timestamp.
 *
 * This function sends AT+CCLK? to the modem, parses the response
 * ("yy/MM/dd,hh:mm:ss±zz"), adjusts for the timezone offset to get UTC, and
 * then converts it to a time_t (epoch) value. It's recommended to have network
 * time synchronization enabled on the modem (AT+CLTS=1) for accurate time.
 *
 * @param handle Pointer to the initialized sim7080g_handle_t.
 * @param epoch_time_utc Pointer to a time_t variable where the UTC epoch time
 * will be stored.
 * @return esp_err_t ESP_OK on success, ESP_FAIL or specific error on failure.
 */
esp_err_t sim7080g_get_epoch_time_utc(const sim7080g_handle_t *handle,
                                      time_t *epoch_time_utc);

/**
 * @brief Synchronizes the SIM7080G module's Real-Time Clock (RTC).
 *
 * This function attempts to synchronize the modem's time using two methods in
 * order:
 * 1. Network Time Synchronization (NITZ) via AT+CLTS:
 * - Enables AT+CLTS=1.
 * - Reboots the module using sim7080g_cycle_cfun() as CLTS changes often
 * require a reboot.
 * - Waits for a period to allow network re-registration.
 * - Checks AT+CCLK?. If the time is valid (not the default "80/01/06") and
 * obtained, the function returns successfully.
 * 2. NTP (Network Time Protocol) Synchronization via AT+CNTP:
 * - This is attempted if NITZ fails or is skipped.
 * - Verifies that a network bearer (PDP context) is active.
 * - Sets the NTP bearer profile ID using AT+CNTPCID.
 * - Configures AT+CNTP with a default NTP server (e.g., "pool.ntp.org"),
 * UTC timezone (0 offset), the active PDP context, and mode 2 (update RTC &
 * output time).
 * - Sends the AT+CNTP execute command to initiate synchronization.
 * - Polls AT+CCLK? for a specified duration to check if the time has been
 * updated, as the actual NTP result (+CNTP: 1) is an Unsolicited Result Code
 * (URC) which is not directly handled by this function's send_at_cmd.
 *
 * @note It is crucial that the modem is connected to the network bearer
 * (e.g., GPRS/LTE) before attempting NTP synchronization.
 * @note If AT+CLTS is used and causes a reboot, the calling application is
 * responsible for ensuring the modem re-establishes its network connection
 * before this function proceeds to check CCLK or attempt NTP. The current
 * implementation includes a placeholder delay for this.
 * @note For robust NTP synchronization, handling the "+CNTP: 1" URC would be
 * more reliable than polling AT+CCLK?.
 * @note This function is essential for operations requiring accurate time, such
 * as SSL/TLS certificate validation for MQTTS.
 *
 * @param handle Pointer to the initialized sim7080g_handle_t structure.
 * @return esp_err_t
 * - ESP_OK if time synchronization was successful (either via NITZ or NTP).
 * - ESP_FAIL if both methods fail or a critical error occurs.
 * - ESP_ERR_INVALID_ARG if the handle is NULL.
 * - ESP_ERR_INVALID_STATE if the network bearer is not active for NTP.
 * - Other esp_err_t codes for specific AT command failures or timeouts.
 */
esp_err_t sim7080g_sync_time(const sim7080g_handle_t *handle);

#ifdef __cplusplus
}
#endif

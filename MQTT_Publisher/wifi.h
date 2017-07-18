#include <net/if.h>
#include <apps/netutils/wifi/slsi_wifi_api.h>

// WIFi
#define STATE_DISCONNECTED          0
#define STATE_CONNECTED             1

#define SLSI_WIFI_SECURITY_OPEN     "open"
#define SLSI_WIFI_SECURITY_WPA2_AES "wpa2_aes"

#define SSID "Dr_Kook"
#define PSK  "1357924680abc"

slsi_security_config_t *getSecurityConfig(char *sec_type,
        char *psk, WiFi_InterFace_ID_t mode);

static int g_connection_state = STATE_DISCONNECTED;
static uint8_t g_join_result  = 0;
static sem_t g_sem_join;

/**
 * Handler for network link up connection event
 *
 *   Sets the global connection state variable and the
 *   result of the network join request.
 */
void networkLinkUpHandler(slsi_reason_t* reason) {
    g_connection_state = STATE_CONNECTED;

    g_join_result = reason->reason_code;
    sem_post(&g_sem_join);
}

/**
 * Handler for network link down connection event
 *
 *   Sets the global connection variable when the access
 *   point is disconnected from the network.
 */
void networkLinkDownHandler(slsi_reason_t* reason) {
    g_connection_state = STATE_DISCONNECTED;

    if (reason) {
        printf("Disconnected from network %s "
               "reason_code: %d %s\n", reason->bssid,
                reason->reason_code,
                reason->locally_generated ?
                        "(locally_generated)": "");
    } else {
        printf("Disconnected from network\n");
    }
}

/**
 * Starts the Wi-Fi interface and request connection to the
 * specified network
 *
 *   Return: Completed successfully or failed
 *
 *   Starts the Wi-Fi interface in Station mode and requests
 *   to join the specified network.
 */
int8_t start_wifi_interface(void) {

    if ( WiFiRegisterLinkCallback(&networkLinkUpHandler,
            &networkLinkDownHandler) ) {
        return SLSI_STATUS_ERROR;
    }

    if ( WiFiStart(SLSI_WIFI_STATION_IF, NULL) ==
            SLSI_STATUS_ERROR ) {
        return SLSI_STATUS_ERROR;
    }

    sem_init(&g_sem_join, 0, 0);

    slsi_security_config_t *security_config =
            getSecurityConfig(SLSI_WIFI_SECURITY_WPA2_AES,
                PSK, SLSI_WIFI_STATION_IF);

    if ( WiFiNetworkJoin((uint8_t*)SSID, strlen(SSID),
            NULL, security_config) == SLSI_STATUS_ERROR ) {
        return SLSI_STATUS_ERROR;
    }

    sem_wait(&g_sem_join);

    if( g_join_result ) {
       return SLSI_STATUS_ERROR;
    }

    free(security_config);
    sem_destroy(&g_sem_join);

    return SLSI_STATUS_SUCCESS;
}

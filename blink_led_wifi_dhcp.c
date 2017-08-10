/*---------------------------------------------------------------------------*/
/* Include Files                                                             */
/*---------------------------------------------------------------------------*/
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <apps/netutils/netlib.h>
#include <apps/netutils/wifi/slsi_wifi_api.h>
#include <iotbus/iotbus_gpio.h>
#include <sys/mount.h>
#include <tinyara/arch.h>
#include <tinyara/config.h>
#include <tinyara/gpio.h>

#include <apps/netutils/dhcpc.h>


/*---------------------------------------------------------------------------*/
/* Defines                                                                   */
/*---------------------------------------------------------------------------*/
#define STATE_DISCONNECTED          0
#define STATE_CONNECTED             1

#define SLSI_WIFI_SECURITY_OPEN     "open"
#define SLSI_WIFI_SECURITY_WPA2_AES "wpa2_aes"

#ifndef APP_PRIORITY
#define APP_PRIORITY 100
#endif

#ifndef APP_STACKSIZE
#define APP_STACKSIZE 2048
#endif

/*---------------------------------------------------------------------------*/
/* User Required Defines                                                     */
/*---------------------------------------------------------------------------*/
#define SSID "lwc2"
#define PSK  ""

#define NET_DEVNAME "wl1"

/*---------------------------------------------------------------------------*/
/* Function Prototypes                                                       */
/*---------------------------------------------------------------------------*/
slsi_security_config_t *getSecurityConfig(char *sec_type, char *psk, WiFi_InterFace_ID_t mode);

/*---------------------------------------------------------------------------*/
/* Global Variables                                                          */
/*---------------------------------------------------------------------------*/
static int g_connection_state = STATE_DISCONNECTED;
static uint8_t g_join_result  = 0;
static sem_t g_sem_join;

/**
 * Handler for network link up connection event
 *
 *   Sets the global connection state variable and the result of the network
 *   join request.
 */
void networkLinkUpHandler(slsi_reason_t* reason) {
    g_connection_state = STATE_CONNECTED;

    g_join_result = reason->reason_code;
    sem_post(&g_sem_join);
}

/**
 * Handler for network link down connection event
 *
 *   Sets the global connection variable when the access point is disconnected
 *   from the network.
 */
void networkLinkDownHandler(slsi_reason_t* reason) {
    g_connection_state = STATE_DISCONNECTED;

    if (reason) {
        printf("Disconnected from network %s reason_code: %d %s\n", reason->bssid, reason->reason_code,
                    reason->locally_generated ? "(locally_generated)": "");
    } else {
        printf("Disconnected from network\n");
    }
}

/**
 * Starts the Wi-Fi interface and request connection to the specified network
 * Return: Completed successfully or failed
 *
 *   Starts the Wi-Fi interface in Station mode and requests to join the
 *   specified network.
 */
int8_t start_wifi_interface(void) {

    if ( WiFiRegisterLinkCallback(&networkLinkUpHandler, &networkLinkDownHandler) ) {
        return SLSI_STATUS_ERROR;
    }

    if ( WiFiStart(SLSI_WIFI_STATION_IF, NULL) == SLSI_STATUS_ERROR ) {
        return SLSI_STATUS_ERROR;
    }

    sem_init(&g_sem_join, 0, 0);

    //slsi_security_config_t *security_config = getSecurityConfig(SLSI_WIFI_SECURITY_WPA2_AES, PSK, SLSI_WIFI_STATION_IF);
    slsi_security_config_t *security_config = getSecurityConfig(SLSI_WIFI_SECURITY_OPEN, PSK, SLSI_WIFI_STATION_IF);

    if ( WiFiNetworkJoin((uint8_t*)SSID, strlen(SSID), NULL, security_config) == SLSI_STATUS_ERROR ) {
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
/**
 * Write the value of gpio
 *
 *   Write the value of given gpio port.
 *
 */
void gpio_write(int port, int value)
{
    char str[4];
    static char devpath[16];
    snprintf(devpath, 16, "/dev/gpio%d", port);
    int fd = open(devpath, O_RDWR);

    ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
    write(fd, str, snprintf(str, 4, "%d", value != 0) + 1);

    close(fd);
}
/**
 * Blinks the LED for the given file descriptor handle
 *
 *   For the given file handle, the value is enabled and disabled at one second
 *   period.
 */
void blink_led(int i) {
    gpio_write(i, 1);
    sleep(1);
    gpio_write(i, 0);
    sleep(1);
    return;
}

/**
 * Enable the LED for the Wi-Fi connectivity state
 *
 *   Turns on LED703 (GPIO16) on the ARTIK 053 Starter Kit.
 */
void enable_wifi_starter_kit_led(void) {
    gpio_write(45, 1);
    return;
}

/**
 * Main function for ARTIK Wi-Fi Blink LED Example
 * Return: Completed successfully or failed
 *
 *   Main function to enable and connect the Wi-Fi interface in Station mode
 *   and blinks the LED on the ARTIK 053 Starter Kit.
 */
int main(int argc, char *argv[]) {

#ifdef CONFIG_CTRL_IFACE_FIFO
    int ret;

    ret = mkfifo(CONFIG_WPA_CTRL_FIFO_DEV_REQ, CONFIG_WPA_CTRL_FIFO_MK_MODE);
    if(ret != 0 && ret != -EEXIST) {
        printf("mkfifo error for %s: %s", CONFIG_WPA_CTRL_FIFO_DEV_REQ, strerror(errno));
    }
    ret = mkfifo(CONFIG_WPA_CTRL_FIFO_DEV_CFM, CONFIG_WPA_CTRL_FIFO_MK_MODE);
    if(ret != 0 && ret != -EEXIST) {
        printf("mkfifo error for %s: %s", CONFIG_WPA_CTRL_FIFO_DEV_CFM, strerror(errno));
    }

        ret = mkfifo(CONFIG_WPA_MONITOR_FIFO_DEV, CONFIG_WPA_CTRL_FIFO_MK_MODE);
    if(ret != 0 && ret != -EEXIST){
        printf("mkfifo error for %s: %s", CONFIG_WPA_MONITOR_FIFO_DEV, strerror(errno));
    }
#endif

#ifdef CONFIG_EXAMPLES_MOUNT
    mount_app_main(0, NULL);
#endif

    if ( start_wifi_interface() == SLSI_STATUS_ERROR ) {
    	printf("Connect Wi-Fi failed. Exit.\n");
        return SLSI_STATUS_ERROR;
    }

    // Add DHCP Code
    bool ipObtained = false;
    struct dhcpc_state state;
    void *dhcp_handle;

    while(!ipObtained) {
      dhcp_handle = dhcpc_open(NET_DEVNAME);
      ret = dhcpc_request(dhcp_handle, &state);
      dhcpc_close(dhcp_handle);

      if (ret != OK) {
         printf("Failed to get IP address\n");
         printf("Try again\n");
         sleep(1);
      } else {
         ipObtained = true;
      }
    }
    netlib_set_ipv4addr(NET_DEVNAME, &state.ipaddr);
    netlib_set_ipv4netmask(NET_DEVNAME, &state.netmask);
    netlib_set_dripv4addr(NET_DEVNAME, &state.default_router);
    printf("IP address  %s\n", inet_ntoa(state.ipaddr));
    sleep(5);

    printf("Connect Wi-Fi success, now blink LED\n");
    // Turn on LED703 (XGPIO20) after successful Wi-Fi connection
    //   on ARTIK 053 Starter Kit
    enable_wifi_starter_kit_led();

    while(1) {
        blink_led(49);
    }
}


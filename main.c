#include "wifi.h"
#include <tinyara/gpio.h>
#include <apps/netutils/mqtt_api.h>
#include <apps/netutils/dhcpc.h>

#define DEFAULT_CLIENT_ID "123456789"
#define SERVER_ADDR "192.168.14.4"
#define RED "RED"
#define BLUE "BLUE"
#define GREEN "GREEN"
#define RED_LED 45
#define GREEN_LED 60
#define BLUE_LED 49
#define RED_ON_BOARD_LED 45
#define NET_DEVNAME "wl1"



// mqtt client handle
mqtt_client_t* pClientHandle = NULL;

// mqtt client parameters
mqtt_client_config_t clientConfig;

//int blinkerValue = 0;
int currentLED = 0;

// mqtt client on connect callback
void onConnect(void* client, int result) {
    printf("mqtt client connected to the server\n");
}

// mqtt client on disconnect callback
void onDisconnect(void* client, int result) {
    printf("mqtt client disconnected from the server\n");
}

// Write the value of given gpio port.
void gpio_write(int port, int value) {
    char str[4];
    static char devpath[16];
    snprintf(devpath, 16, "/dev/gpio%d", port);
    int fd = open(devpath, O_RDWR);

    ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
    write(fd, str, snprintf(str, 4, "%d", value != 0) + 1);

    close(fd);
}

void ledSignal(void) {
    switch (currentLED) {
    case RED_LED: {
        gpio_write(GREEN_LED,0);
        gpio_write(BLUE_LED, 0);
        gpio_write(RED_LED, 1);
    }
    break;
    case GREEN_LED: {
        gpio_write(BLUE_LED,0);
        gpio_write(RED_LED, 0);
        gpio_write(GREEN_LED, 1);
    }
    break;
    case BLUE_LED: {
        gpio_write(RED_LED,0);
        gpio_write(GREEN_LED, 0);
        gpio_write(BLUE_LED, 1);
    }
    break;
    default: {
        gpio_write(RED_LED, 0);
        gpio_write(BLUE_LED, 0);
        gpio_write(GREEN_LED, 0);
    }
    break;
    }
    return;
}

// mqtt client on message callback
void onMessage(void *client, mqtt_msg_t *msg) {
    printf("Received message\n");
    printf("Topic: %s\n", msg->topic);
    printf("Message: %s\n", msg->payload);

    // Check if the message received is RED
    int result = strcmp(msg->payload, RED);
    if (result == 0) {
        if (currentLED != RED_LED) {
            currentLED = RED_LED;
            ledSignal();
        }
        printf("Red LED\n");
        return;
    }

    // Check if the message received is GREEN
    result = strcmp(msg->payload, GREEN);
    if (result == 0) {
        if (currentLED != GREEN_LED) {
            currentLED = GREEN_LED;
            ledSignal();
        }
        printf("Green LED\n");
        return;
    }

    // Check if the message received is BLUE
    result = strcmp(msg->payload, BLUE);
    if (result == 0) {
        if (currentLED != BLUE_LED) {
            currentLED = BLUE_LED;
            ledSignal();
        }
        printf("Blue LED\n");
        return;
    }

    currentLED = 0;
    ledSignal();    // Turn off all LEDs
}

// Utility function to configure mqtt client
void initializeConfigUtil(void) {
    uint8_t macId[IFHWADDRLEN];
    int result = netlib_getmacaddr("wl1", macId);
    if (result < 0) {
        printf("Get MAC Address failed. Assigning \
                Client ID as 123456789");
        clientConfig.client_id =
                DEFAULT_CLIENT_ID; // MAC id Artik 053
    } else {
    printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
            ((uint8_t *) macId)[0],
            ((uint8_t *) macId)[1], ((uint8_t *) macId)[2],
            ((uint8_t *) macId)[3], ((uint8_t *) macId)[4],
            ((uint8_t *) macId)[5]);
    char buf[12];
    sprintf(buf, "%02x%02x%02x%02x%02x%02x",
            ((uint8_t *) macId)[0],
            ((uint8_t *) macId)[1], ((uint8_t *) macId)[2],
            ((uint8_t *) macId)[3], ((uint8_t *) macId)[4],
            ((uint8_t *) macId)[5]);
    clientConfig.client_id = buf; // MAC id Artik 053
    printf("Registering mqtt client with id = %s\n", buf);
    }
    clientConfig.on_connect = (void*) onConnect;
    clientConfig.on_disconnect = (void*) onDisconnect;
    clientConfig.on_message = (void*) onMessage;
}

int main(int argc, char *argv[]) {

    bool wifiConnected = false;
    gpio_write(RED_ON_BOARD_LED, 1); // Turn on on board Red LED to indicate no WiFi connection is established

#ifdef CONFIG_CTRL_IFACE_FIFO
    int ret;

    while(!wifiConnected) {
        ret = mkfifo(CONFIG_WPA_CTRL_FIFO_DEV_REQ,
                CONFIG_WPA_CTRL_FIFO_MK_MODE);
        if (ret != 0 && ret != -EEXIST) {
            printf("mkfifo error for %s: %s",
                    CONFIG_WPA_CTRL_FIFO_DEV_REQ,
                    strerror(errno));
        }
        ret = mkfifo(CONFIG_WPA_CTRL_FIFO_DEV_CFM,
                CONFIG_WPA_CTRL_FIFO_MK_MODE);
        if (ret != 0 && ret != -EEXIST) {
            printf("mkfifo error for %s: %s",
                    CONFIG_WPA_CTRL_FIFO_DEV_CFM,
                    strerror(errno));
        }

        ret = mkfifo(CONFIG_WPA_MONITOR_FIFO_DEV,
                CONFIG_WPA_CTRL_FIFO_MK_MODE);
        if (ret != 0 && ret != -EEXIST) {
            printf("mkfifo error for %s: %s",
                    CONFIG_WPA_MONITOR_FIFO_DEV,
                    strerror(errno));
        }
    #endif

        if (start_wifi_interface() == SLSI_STATUS_ERROR) {
            printf("Connect Wi-Fi failed. Try Again.\n");
        }
        else {
            wifiConnected = true;
            gpio_write(RED_ON_BOARD_LED, 0); // Turn off Red LED to indicate WiFi connection is established
        }
    }

    printf("Connect to Wi-Fi success\n");

    bool mqttConnected = false;
    bool ipObtained = false;
    printf("Get IP address\n");

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
        }
        else {
            ipObtained = true;
        }
    }
    netlib_set_ipv4addr(NET_DEVNAME, &state.ipaddr);
    netlib_set_ipv4netmask(NET_DEVNAME, &state.netmask);
    netlib_set_dripv4addr(NET_DEVNAME, &state.default_router);

    printf("IP address  %s\n", inet_ntoa(state.ipaddr));

    sleep(5);

    // Connect to the WiFi network for Internet connectivity
    printf("mqtt client tutorial\n");

    // Initialize mqtt client
    initializeConfigUtil();

    pClientHandle = mqtt_init_client(&clientConfig);
    if (pClientHandle == NULL) {
        printf("mqtt client handle initialization fail\n");
        return 0;
    }

    while (mqttConnected == false ) {
        sleep(2);
        // Connect mqtt client to server
        int result = mqtt_connect(pClientHandle,
                SERVER_ADDR, 1883, 60);
        if (result < 0) {
            printf("mqtt client connect to server fail\n");
            continue;
        }
        mqttConnected = true;
    }

    bool mqttSubscribe = false;

    // Subscribe to topic of interest
    while (mqttSubscribe == false ) {
        sleep(2);
        int result = mqtt_subscribe(pClientHandle,
                "color", 0); //topic - color, QOS - 0
        if (result < 0) {
            printf("mqtt client subscribe to topic \
                    failed\n");
            continue;
        }
        mqttSubscribe = true;
        printf("mqtt client Subscribed to the topic successfully\n");
    }

    while(1) {
    }
}

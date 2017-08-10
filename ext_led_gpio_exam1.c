#include <fcntl.h>
#include <tinyara/gpio.h>

static void gpio_write(int port, int value)
{
  char str[4];
  static char devpath[16];
  snprintf(devpath, 16, "/dev/gpio%d", port);
  int fd = open(devpath, O_RDWR);

  ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
  write(fd, str, snprintf(str, 4, "%d", value != 0) + 1);

  close(fd);
}

int main(int argc, FAR char *argv[]){
  //tash_cmd_install("sensor", sensorbd_main, 1);
  int i;

  for (i = 0; i < 10; i++) {
    gpio_write(51, 1);
    printf("LED On\n");
    up_mdelay(500);
    gpio_write(51, 0);
    printf("LED Off\n");
    up_mdelay(500);
  }

  return 0;
}

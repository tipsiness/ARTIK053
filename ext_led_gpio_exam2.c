#include <fcntl.h>
#include <tinyara/gpio.h>

int main(int argc, FAR char *argv[]){
  //tash_cmd_install("sensor", sensorbd_main, 1);
  int i;

	int fd = open("/dev/gpio51", O_RDWR);

  ioctl(fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);

  for (i = 0; i < 10; i++) {
    //gpio_write(51, 1);
		write(fd, "1", 1);
    printf("LED On\n");
    up_mdelay(500);
    //gpio_write(51, 0);
    printf("LED Off\n");
		write(fd, "0", 1);
    up_mdelay(500);
  }

	close(fd);

  return 0;
}

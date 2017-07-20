#include <stdio.h>
#include <apps/shell/tash.h>
#include <fcntl.h>
#include <tinyara/gpio.h>

#include <tinyara/pwm.h>

int main(int argc, FAR char *argv[]) {
  int i;
  int fd1;
  struct pwm_info_s pwm_info;

  fd1 = open("/dev/pwm0", O_RDWR);
  pwm_info.frequency = 1000;

  while (1) {
    for (i = 1; i < 100; i+=3) {
      printf("brightness %d\n", i);
      pwm_info.duty = i * 65536 / 100;
      ioctl(fd1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
      ioctl(fd1, PWMIOC_START);
      up_mdelay(100);
    }

    for (i = 1; i < 100; i+=3) {
      printf("brightness %d\n", i);
      pwm_info.duty = (100-i) * 65536 / 100;
      ioctl(fd1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
      ioctl(fd1, PWMIOC_START);
      up_mdelay(100);
    }
  } // end while

  ioctl(fd1, PWMIOC_STOP);
  close(fd1);

  return 0;
}

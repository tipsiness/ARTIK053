#include <stdio.h>
#include <apps/shell/tash.h>

#include <errno.h>
#include <fcntl.h>
#include <tinyara/analog/adc.h>
#include <tinyara/analog/ioctl.h>

int main(int argc, FAR char *argv[]){
  int fd, ret;
  struct adc_msg_s sample;
  size_t readsize;
  ssize_t nbytes;

  fd = open("/dev/adc0", O_RDONLY);

  printf("ADC Data:\n");

  for (;;) {
    ret = ioctl(fd, ANIOC_TRIGGER, 0);

    readsize = sizeof(struct adc_msg_s);
    nbytes = read(fd, &sample, readsize);

    int nsamples = sizeof(struct adc_msg_s);
    if (sample.am_channel == 0) {
      printf("ADC Channel: %d, value: %d\n",sample.am_channel, sample.am_data);
    }
    usleep(500);
  }
  close(fd);
}

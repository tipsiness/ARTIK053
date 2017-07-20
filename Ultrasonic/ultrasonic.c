#include <stdio.h>
#include <apps/shell/tash.h>
#include <tinyara/gpio.h>
#include <tinyara/config.h>

#define GPIO_FUNC_SHIFT 13
#define GPIO_INPUT (0x0 << GPIO_FUNC_SHIFT)
#define GPIO_OUTPUT (0x1 << GPIO_FUNC_SHIFT)

#define GPIO_PORT_SHIFT 3
#define GPIO_PORTG1 (0x5 << GPIO_PORT_SHIFT)

#define GPIO_PIN_SHIFT 0
#define GPIO_PIN4 (0x4 << GPIO_PIN_SHIFT)
#define GPIO_PIN2 (0x2 << GPIO_PIN_SHIFT)

#define GPIO_PUPD_SHIFT 11
#define GPIO_PULLDOWN (0x1 << GPIO_PUPD_SHIFT)
#define GPIO_PULLUP (0x3 << GPIO_PUPD_SHIFT)

int main(int argc, FAR char *argv[]) {
  float distance;
  long count;

  uint32_t cfgcon_trig; //trig
  uint32_t cfgcon_echo; //echo

  printf("Distance Measure..\n");

  cfgcon_trig = GPIO_OUTPUT | GPIO_PORTG1 | GPIO_PIN4; //XGPIO12
  cfgcon_echo = GPIO_INPUT | GPIO_PORTG1 | GPIO_PIN2;  //XGPIO10

  s5j_configgpio(cfgcon_trig);
  s5j_configgpio(cfgcon_echo);

  while (1) {
    count = 0;

    s5j_gpiowrite(cfgcon_trig, 0); up_udelay(2);
    s5j_gpiowrite(cfgcon_trig, 1); up_udelay(10);
    s5j_gpiowrite(cfgcon_trig, 0);

    while (s5j_gpioread(cfgcon_echo) == 0);
    while (s5j_gpioread(cfgcon_echo) == 1) {
      count++;
      up_udelay(1);
    }

    distance = count / 29.0 / 2.0;
    printf("Duration: %d us, Distance: %.2f cm\n", count, distance);

    up_mdelay(2000);
  }

  return 0;
}

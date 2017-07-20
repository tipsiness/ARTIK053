#include <stdio.h>
#include <apps/shell/tash.h>

#include <tinyara/gpio.h>
#include <tinyara/config.h>

#define GPIO_FUNC_SHIFT 13
#define GPIO_INPUT (0x0 << GPIO_FUNC_SHIFT)
#define GPIO_OUTPUT (0x1 << GPIO_FUNC_SHIFT)

#define GPIO_PORT_SHIFT 3
#define GPIO_PORTG2 (0x6 << GPIO_PORT_SHIFT)

#define GPIO_PIN_SHIFT 0
#define GPIO_PIN6 (0x6 << GPIO_PIN_SHIFT)

unsigned char get_value(void);
uint32_t cfgcon_out, cfgcon_in;

int main(int argc, FAR char *argv[]) {
  unsigned char data[5];
  int i, j, val;
  float temp, humi;
  int tmp;

  printf("Temperature & Humidity (DHT11)\n");

  cfgcon_out = GPIO_OUTPUT | GPIO_PORTG2 | GPIO_PIN6;
  cfgcon_in = GPIO_INPUT | GPIO_PORTG2 | GPIO_PIN6;

  while (1) {
    s5j_configgpio(cfgcon_out);

    s5j_gpiowrite(cfgcon_out, 1);
    up_mdelay(1);

    s5j_gpiowrite(cfgcon_out, 0);
    up_mdelay(18); // for DHT11

    s5j_gpiowrite(cfgcon_out, 1);
    up_udelay(40);

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    s5j_configgpio(cfgcon_in);

    up_udelay(160);

    for (i = 0; i < 5; i++) {
      for (j = 0; j < 8; j++) {
        if (j < 7) {
			data[i] |= get_value();
			data[i] = data[i] << 1;
				} else {
			data[i] |= get_value();
        }
      }
    }

    tmp = 0;

    for (i = 0; i < 4; i++) {
      tmp += data[i];
    }
    if (data[4] == (tmp & 0xFF)) {
      humi = data[0] + (float)(data[1]/100.0);
      temp = data[2] + (float)(data[3]/100.0);

      printf("Temperature: %.1f, Humidity: %.1f\n", temp, humi);
    } else {
      printf("Parity Error\n");
      printf("data[0]: %d, data[1]: %d, data[2]: %d, data[3]: %d, \
              data[4]: %d\n", data[0], data[1], data[2], data[3], data[4]);
    }
    up_mdelay(4000);
  }

  return 0;
}

unsigned char get_value(void) {
  unsigned char va = 0;
  int state;

  while (s5j_gpioread(cfgcon_in) == 0);

  up_udelay(28);

  if (s5j_gpioread(cfgcon_in) == 1) {
    va |= 1;

    while (s5j_gpioread(cfgcon_in) == 1);
  }

  return va;
}

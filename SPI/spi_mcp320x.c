#include <stdio.h>
#include <apps/shell/tash.h>
#include <tinyara/spi/spi.h>
#include <arpa/inet.h>

static void spi_transferb(int port, int frequency, int bits, int conf, 
                          unsigned char *tx_buf, 
                          unsigned char *rx_buf);

unsigned char msbtolsb(unsigned char *data);

static struct spi_dev_s *spi_dev;

int main(int argc, FAR char *argv[]){
  int port = 0;
  int freq = 1000000;
  int bits = 8;
  int conf = 0;
  int adcValue, i, j;
  int adc_res = 4096; //12bit ADC
  float ref_vol = 3.3;
  float adc_vol;

  unsigned char tx_buf[] = {0x06, 0, 0};
  unsigned char rx_buf[3];
  unsigned char reversed_rx_data[2];

  spi_dev = up_spiinitialize(port);

  up_mdelay(500);

  while (1) {
    spi_transferb(port, freq, bits, conf, tx_buf, rx_buf);

    reversed_rx_data[1] = msbtolsb(&rx_buf[1]);
    reversed_rx_data[0] = msbtolsb(&rx_buf[0]);

    adcValue = (reversed_rx_data[1] * 256) + reversed_rx_data[0];
    adc_vol = (ref_vol / adc_res) * adcValue;

    printf("ADC Value: %d, Voltage: %.1f(V)\n", adcValue, adc_vol);
    up_mdelay(2000);
  }

  return 0;
}

static void spi_transferb(int port, int frequency, int bits, int conf, 
                          unsigned char *tx_buf, unsigned char *rx_buf) 
{
  SPI_SETFREQUENCY(spi_dev, frequency);
  SPI_SETBITS(spi_dev, bits);
  SPI_SETMODE(spi_dev, conf);

  SPI_SELECT(spi_dev, port, true);
  SPI_SNDBLOCK(spi_dev, tx_buf, 3);
  SPI_RECVBLOCK(spi_dev, rx_buf, 3);
  SPI_SELECT(spi_dev, port, false);

  return;
}

unsigned char msbtolsb(unsigned char *data) {
  unsigned char oldval = *data;
  int s = sizeof(oldval) * 8;
  int i, x, y, p;
  int var = 0;

  for (i = 0; i < (s / 2); i++) {
    p = s - i - 1;
    x = oldval & (1 << p);
    x = x >> p;

    y = oldval & (1 << i);
    y = y >> i;

    var = var | (x << i);
    var = var | (y << p);
  }
  return var;
}

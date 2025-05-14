#include <stdio.h>
#include <unistd.h>
#include <bcm2835.h>
#include "drivers/led_indicator.h"
#include "common/common_defs.h"

int main(void) {
  if (!bcm2835_init()) return 1;
  led_indicator_init(PIN_LED_1, PIN_LED_2);
  int last_status = -1;

  while (1) {
    int st = last_status;
    FILE *f = fopen("/run/elevator_status","r");
    if (f) {
      fscanf(f, "%d", &st);
      fclose(f);
    }
    if (st != last_status) {
      led_indicator_set_status(st);
      last_status = st;
    }
    led_indicator_update();
    usleep(10000);  // 100 Hz blink‐tick
  }
  return 0;
}
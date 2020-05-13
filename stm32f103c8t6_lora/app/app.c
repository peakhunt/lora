#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "app.h"
#include "event_dispatcher.h"
#include "shell.h"
#include "mainloop_timer.h"
#include "blinky.h"
#include "sx127x.h"
#include "sx127x_test.h"

void
app_init_f(void)
{
  event_dispatcher_init();
  mainloop_timer_init();
}

void
app_init(void)
{
  blinky_init();
  sx127x_init();
  sx127x_test_init();

  __disable_irq();
  shell_init();
  __enable_irq();
}

void
app_loop(void)
{
  while(1)
  {
    event_dispatcher_dispatch();
  }
}

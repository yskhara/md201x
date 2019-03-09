#ifndef _LED_H
#define _LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#define LED_DURATION 10

#define LED_STAT_ON_DUR 2900
#define LED_STAT_OFF_DUR 100

void led_on(void);
void led_process(void);

#ifdef __cplusplus
}
#endif

#endif

#pragma once
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define RISING 3
using std::min;
using std::max;
template <typename T, typename L, typename H> T constrain(T val,L lo,H hi) { return max((T)lo, min(val,(T)hi)); }
extern unsigned long now_ms;
extern int adc_input;
extern int pwm_output;
extern int digital_pin_output[20];
extern bool encoder_b_high;
extern void (*irq_callback)();
inline unsigned long millis() {return now_ms;}
inline void pinMode(int,int) {}
inline int analogRead(int) {return adc_input;}
inline void digitalWrite(int pin,int level) {if(pin<20) digital_pin_output[pin]=level;}
inline int digitalRead(int pin) {return (pin==4 && encoder_b_high)?1:0;}
inline void analogWrite(int,int level) {pwm_output=level;}
inline int digitalPinToInterrupt(int pin) {return pin;}
inline void attachInterrupt(int,void(*cb)(),int) {irq_callback=cb;}
inline void noInterrupts() {}
inline void interrupts() {}

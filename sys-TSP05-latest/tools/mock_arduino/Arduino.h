#pragma once
// HOST-ONLY syntax shim; not a replacement for compiling with the Arduino AVR core.
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <algorithm>
#define INPUT 0
#define INPUT_PULLUP 2
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define RISING 3
#define A0 14
#define A1 15
#define A2 16
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
using std::min;
using std::max;
template <typename T, typename L, typename H> T constrain(T value, L l, H h) { return max((T)l, min(value, (T)h)); }
inline void pinMode(int,int) {}
inline int digitalRead(int) { return 0; }
inline int analogRead(int) { return 512; }
inline void digitalWrite(int,int) {}
inline void analogWrite(int,int) {}
inline void attachInterrupt(int, void (*)(), int) {}
inline int digitalPinToInterrupt(int v) {return v;}
inline void noInterrupts() {}
inline void interrupts() {}
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline long map(long x,long a,long b,long c,long d) {return (x-a)*(d-c)/(b-a)+c;}
struct DummySerial {
    void begin(int){}
    int available(){return 0;}
    int availableForWrite(){return 64;}
    int read(){return 0;}
    size_t write(char){return 1;}
    size_t write(unsigned char){return 1;}
    template<typename T> void print(T){}
    template<typename T> void print(T,int){}
};
extern DummySerial Serial;

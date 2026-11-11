#include <cstdio>
#include <cassert>
#include "Arduino.h"
#include "DcMotor.h"
#include "SharpIRSensor.h"
unsigned long now_ms=0;
int adc_input=255;
int pwm_output=0;
int digital_pin_output[20]={0};
bool encoder_b_high=true;
void (*irq_callback)()=nullptr;
void advance(unsigned long ms) { now_ms+=ms; }
int main(){
  DCMotor dc(6,8,7,3,4);
  dc.begin();
  dc.moveRelativeDegrees(90.0f);
  advance(200);
  dc.update();
  assert(dc.getActiveTargetRPM()>0 && pwm_output>0);
  assert(digital_pin_output[8]==1 && digital_pin_output[7]==0);
  for(int i=0;i<51;i++){irq_callback();dc.update();}
  assert(dc.positionReached()); // estimated RPM is still 0 from first sample
  assert(pwm_output==0);
  for(int i=0;i<20;i++){irq_callback();advance(20);dc.update();}
  assert(pwm_output==0); // even after actual encoder overshoot: NO reversal
  dc.moveRelativeDegrees(-90);
  encoder_b_high=false;
  advance(200);dc.update();
  assert(dc.getActiveTargetRPM()<0);
  for(int i=0;i<51;i++){irq_callback();dc.update();}
  assert(pwm_output==0);
  advance(300); dc.update(); assert(pwm_output==0);
  puts("PASS: DC relative +90 and -90 moves stop; overshoot never reverses");

  SharpIRSensor sensor(1);
  sensor.begin();
  advance(50); sensor.update();
  int normal=sensor.getDistanceCm();
  assert(normal>0);
  adc_input=90;
  advance(50);sensor.update();
  assert(abs(sensor.getDistanceCm()-normal)<=1); // single spike rejected
  adc_input=255;
  advance(50);sensor.update();
  assert(abs(sensor.getDistanceCm()-normal)<=1);
  adc_input=90;
  for(int j=0;j<3;j++){advance(50);sensor.update();}
  assert(sensor.getDistanceCm()>normal+12);
  puts("PASS: Sharp rejects isolated spike and accepts sustained new distance");
  return 0;
}

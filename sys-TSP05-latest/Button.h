#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

// Debounced push button, polled from loop() (no interrupt, so it can share
// a board with other interrupt users). Based on the Button1Pressed() logic
// from vamsig_TSP02: a press is reported once the pin has read the pressed
// level for longer than DEBOUNCE_MS, and the button must be released before
// another press is reported.
class Button {
public:
    // pin: Arduino digital pin
    // activeHigh: true  = button to 5V with external pull-down (pressed = HIGH)
    //             false = button to GND, internal pull-up (pressed = LOW)
    Button(uint8_t pin, bool activeHigh = true);

    void begin();

    // Call repeatedly from loop()
    void update();

    // True once per debounced press (cleared when called)
    bool wasPressed();

    // Current debounced state
    bool isPressed() const;

    // Number of debounced presses since begin()
    unsigned long getPressCount() const;

private:
    uint8_t pin;
    bool activeHigh;

    bool locked = false;        // pressed and already reported
    bool pressEvent = false;
    unsigned long pressCount = 0;
    unsigned long lastReleaseTime = 0;

    static constexpr unsigned long DEBOUNCE_MS = 50;

    bool readPressed() const;
};

#endif

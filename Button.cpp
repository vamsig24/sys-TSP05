#include "Button.h"

// ====================================
// Constructor
// ====================================

Button::Button(uint8_t pin, bool activeHigh)
    : pin(pin),
      activeHigh(activeHigh)
{
}

// ====================================
// Initialization
// ====================================

void Button::begin() {

    pinMode(pin, activeHigh ? INPUT : INPUT_PULLUP);

    locked = readPressed();  // ignore a button held during startup
    pressEvent = false;
    pressCount = 0;
    lastReleaseTime = millis();
}

// ====================================
// Debounce
// ====================================

bool Button::readPressed() const {
    return digitalRead(pin) == (activeHigh ? HIGH : LOW);
}

void Button::update() {

    if (readPressed()) {

        // Report the press once it has been stable for DEBOUNCE_MS
        if (!locked && millis() - lastReleaseTime > DEBOUNCE_MS) {
            locked = true;
            pressEvent = true;
            pressCount++;
        }

    } else {

        // Any released reading restarts the debounce window
        locked = false;
        lastReleaseTime = millis();
    }
}

// ====================================
// Getters
// ====================================

bool Button::wasPressed() {
    bool result = pressEvent;
    pressEvent = false;
    return result;
}

bool Button::isPressed() const {
    return locked;
}

unsigned long Button::getPressCount() const {
    return pressCount;
}

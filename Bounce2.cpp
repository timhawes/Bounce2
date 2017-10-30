// Please read Bounce3.h for information about the liscence and authors

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Bounce2.h"

Bounce::Bounce() {
  interval_millis  = 10;
}

void Bounce::attach(int pin) {
  this->pin = pin;
  state.debouncedState = digitalRead(pin); // Noticed that first read after pinMode(pin, INPUT_PULLUP) isn't always correct....
  state.debouncedState = digitalRead(pin); // ....So, read it twice
  state.unstableState = state.debouncedState;
  state.stateChanged = 0;
  state.timerActive = 1;
  previous_millis = millis();
}

void Bounce::attach(int pin, int mode) {
  #if defined(ARDUINO_STM_NUCLEO_F103RB) || defined(ARDUINO_GENERIC_STM32F103C)
    pinMode(pin, (WiringPinMode)mode);
#else
    pinMode(pin, mode);
#endif
  this->attach(pin);
}

void Bounce::interval(uint16_t interval_millis)
{
  this->interval_millis = interval_millis;
}

bool Bounce::update()
{
  // If we're in timing mode, check if interval has expired
  if ((boolean)state.timerActive) {
    if (millis() - previous_millis >= interval_millis) {
      state.timerActive = 0;
    }
  }
  
#ifdef BOUNCE_LOCK_OUT
  state.stateChanged = 0;  // Clear Changed State Flag - will be reset if we confirm a button state change.

  // Ignore everything if we are locked out
  if (!(boolean)state.timerActive) {
    uint8_t readState = digitalRead(pin);
    if (readState != state.debouncedState) {
      previous_millis = millis();
      state.timerActive = 1;
      state.debouncedState = readState;
      state.stateChanged = 1;
    }
  }
  return state.stateChanged;

#elif defined (BOUNCE_WITH_PROMPT_DETECTION)  
  uint8_t readState = digitalRead(pin);  // Read the state of the switch port into a temporary variable.  
  state.stateChanged = 0;  // Clear Changed State Flag - will be reset if we confirm a button state change.

  if (readState != state.debouncedState) {   
    // We have seen a change from the current button state.
    if (!(boolean)state.timerActive) {
      // We have passed the time threshold, so a new change of state is allowed.
      // set the STATE_CHANGED flag and the new DEBOUNCED_STATE.
      // This will be prompt as long as there has been greater than interval_misllis ms since last change of input.
      // Otherwise debounced state will not change again until bouncing is stable for the timeout period.
      state.debouncedState = readState;
      state.stateChanged = 1;
    }
  }

  // If the readState is different from previous readState, reset the debounce timer - as input is still unstable
  // and we want to prevent new button state changes until the previous one has remained stable for the timeout.
  if (readState != state.unstableState) {
    // Update Unstable Bit to macth readState
    state.unstableState = readState;
    state.timerActive = 1;
    previous_millis = millis();
  }
  // return just the sate changed bit
  return state.stateChanged;
  
#else
  uint8_t readState = digitalRead(pin);  // Read the state of the switch port into a temporary variable.
  state.stateChanged = 0;  // Clear Changed State Flag - will be reset if we confirm a button state change.

  // If the reading is different from last reading, reset the debounce counter
  if (readState != state.unstableState) {
    state.unstableState = readState;
    state.timerActive = 1;
    previous_millis = millis();
  } else if (!(boolean)state.timerActive) {
    // We have passed the threshold time, so the input is now stable
    // If it is different from last state, set the STATE_CHANGED flag
    if (readState != state.debouncedState) {
      state.debouncedState = readState;
      state.stateChanged = 1;
    }
  }
  return state.stateChanged;
#endif
}

bool Bounce::read()
{
  return state.debouncedState;
}

bool Bounce::rose()
{
  return state.debouncedState & state.stateChanged;
}

bool Bounce::fell()
{
  return ~state.debouncedState & state.stateChanged;
}

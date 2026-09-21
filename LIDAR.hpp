#pragma once
 
#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>
 
class Lidar {
public:
  static constexpr int SEARCH_START = 48;    // where address scanning begins
  static constexpr int SEARCH_END   = 119;   // last valid 7-bit addr
  static constexpr int ADDRESS_NONE = 0;     // sentinel: none found
  static constexpr int OUT_OF_RANGE_MM = -1; // sentinel: no valid reading yet
  static constexpr size_t MAX_SENSORS = 8;   // registry capacity
 
  explicit Lidar(uint8_t enablePin)
    : _enablePin(enablePin), _address(ADDRESS_NONE), _lastRangeMm(OUT_OF_RANGE_MM) {
    if (registryCount() < MAX_SENSORS) {
      registry()[registryCount()++] = this;
    }
  }
 
  void beginDisabled() {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, LOW);
  }
 
  bool enable(uint8_t rangeIntervalMs = 100, uint16_t powerUpDelayMs = 50) {
    digitalWrite(_enablePin, HIGH);
    delay(powerUpDelayMs);
 
    _address = findAvailableAddress(searchCursor());
    if (_address == ADDRESS_NONE) {
      Serial.print(F("Lidar: no free I2C address found for enable pin "));
      Serial.println(_enablePin);
      return false;
    }
 
    _sensor.init();
    _sensor.configureDefault();
    _sensor.setTimeout(250);
    _sensor.setAddress((uint8_t)_address);
    _sensor.startRangeContinuous(rangeIntervalMs);
 
    // Next sensor's search resumes after the address we just claimed.
    searchCursor() = _address + 1;
 
    delay(powerUpDelayMs);
    return true;
  }
 
  int read() {
    _lastRangeMm = (int)_sensor.readRangeContinuousMillimeters();
    if (_sensor.timeoutOccurred() || _sensor.readRangeStatus() != 0) {
      _lastRangeMm = OUT_OF_RANGE_MM;
    }
    return _lastRangeMm;
  }
 
  int lastRange() const { return _lastRangeMm; }
 
  bool isOutOfRange() const { return _lastRangeMm == OUT_OF_RANGE_MM; }
 
  // Prints the last reading as "OUT OF RANGE" instead of the raw
  // sentinel number when there's no valid sample.
  void printRange(Print& out) const {
    if (isOutOfRange()) {
      out.print(F("OUT OF RANGE"));
    } else {
      out.print(_lastRangeMm);
    }
  }
 
  bool timeoutOccurred() {
    return _sensor.timeoutOccurred();
  }
 
  int address() const { return _address; }
 
  static int findAvailableAddress(int startAddress = SEARCH_START) {
    for (int addr = startAddress; addr <= SEARCH_END; addr++) {
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
      if (error == 2) {
        return addr;
      }
    }
    return ADDRESS_NONE;
  }
 
  // Brings up every Lidar instance constructed so far, in the correct
  // order: disable all first (so none collide while still at the
  // shared default address), then enable+address them one at a time.
  // Call this once in setup(), after Wire.begin().
  static bool initAll(uint8_t rangeIntervalMs = 100) {
    for (size_t i = 0; i < registryCount(); i++) {
      registry()[i]->beginDisabled();
    }
    bool sensorsWorking = true;
    for (size_t i = 0; i < registryCount(); i++) {
      if (!registry()[i]->enable(rangeIntervalMs)) {
        sensorsWorking = false;
      }
    }
    return sensorsWorking;
  }
 
private:
  static int& searchCursor() {
    static int cursor = SEARCH_START;
    return cursor;
  }
 
  static Lidar** registry() {
    static Lidar* sensors[MAX_SENSORS] = { nullptr };
    return sensors;
  }
  static size_t& registryCount() {
    static size_t count = 0;
    return count;
  }
 
  VL6180X _sensor;
  uint8_t _enablePin;
  int _address;
  int _lastRangeMm;
};

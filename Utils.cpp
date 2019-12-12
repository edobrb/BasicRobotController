#include "Arduino.h"

bool isExpired(long start_time, long duration) {
  return (micros() - start_time) >= duration;
}
long now() {
  return micros();
}
long seconds(float s) {
  return (long)(s * 1000000);
}

#include "Arduino.h"

#define SIGN_EXPR(v) (v>0?1:(v<0?-1:0))
bool isExpired(long start_time, long duration) {
  return (micros() - start_time) >= duration;
}
long now() {
  return micros();
}
long seconds(float s) {
  return (long)(s * 1000000);
}
int sign(double v) {
  return SIGN_EXPR(v);
}
int sign(float v) {
  return SIGN_EXPR(v);
}
int sign(int v) {
  return SIGN_EXPR(v);
}
int sign(long v) {
  return SIGN_EXPR(v);
}

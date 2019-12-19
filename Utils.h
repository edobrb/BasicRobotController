#ifndef __UTILS__
#define __UTILS__

bool isExpired(long start_time, long duration);
long now();
long seconds(float s);

int sign(double v);
int sign(float v);
int sign(int v);
int sign(long v);

int between(int min, int max, int v);


#endif

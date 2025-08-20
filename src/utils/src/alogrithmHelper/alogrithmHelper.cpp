#include "alogrithmHelper/alogrithmHelper.hpp"

float sign0(float x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

float distance(float x0, float y0, float x1, float y1)
{
    return hypot(x1 - x0, y1 - y0);   
}

float distanceToLine(float pX, float pY, float x0, float y0, float x1, float y1)
{
  float A = pX - x0;
  float B = pY - y0;
  float C = x1 - x0;
  float D = y1 - y0;

  float dot = A * C + B * D;
  float len_sq = C * C + D * D;
  float param = dot / len_sq;

  float xx, yy;

  if (param < 0) {
    xx = x0;
    yy = y0;
  } else if (param > 1) {
    xx = x1;
    yy = y1;
  } else {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return distance(pX, pY, xx, yy);
}
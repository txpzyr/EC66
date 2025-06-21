#include "helper.h"

float fminf3(float x, float y, float z){
  /// Returns minimum of x, y, z ///
  return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}
float fmaxf3(float x, float y, float z){
  /// Returns maximum of x, y, z ///
  return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}


float fast_fmaxf(float x, float y){
  /// Returns maximum of x, y ///
  return (((x)>(y))?(x):(y));
}

float fast_fminf(float x, float y){
  /// Returns minimum of x, y ///
  return (((x)<(y))?(x):(y));
}
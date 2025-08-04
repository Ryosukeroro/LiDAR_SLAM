#ifndef POINT_H
#define POINT_H

#include <stdexcept>

struct Point {
  double x;
  double y;
  
  static const int DIM = 2;
  
  double operator[](int index) const {
  if (index == 0) return x;
  else if (index == 1) return y;
  else throw std::out_of_range("Point index out of range");
  }
  
  double& operator[](int index) {
  if (index == 0) return x;
  else if (index == 1) return y;
  else throw std::out_of_range("Point index out of range");
  }
};

#endif // POINT_H

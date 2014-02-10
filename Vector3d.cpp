#include "Vector3d.h"
#include <math.h>

Vector3d::Vector3d(double a, double b, double c) {
  x = a;
  y = b;
  z = c;
  
}

double Vector3d:: 
dot(const Vector3d& v)
{
  return this->X()*v.X() + this->Y()*v.Y() + this->Z()*v.Z(); 
} 

double Vector3d:: 
length()
{
  return sqrt(X()*X() + Y()*Y() + Z()*Z()); 
}

double Vector3d::
lsquared()
{
    return X()*X() + Y()*Y() + Z()*Z();
}

Vector3d operator+(const Vector3d& v1, const Vector3d& v2)
{
  return Vector3d(v1.X() + v2.X(), v1.Y() + v2.Y(), v1.Z() + v2.Z());  
}
 
Vector3d operator-(const Vector3d& v1, const Vector3d& v2)
{
  return Vector3d(v1.X() - v2.X(), v1.Y() - v2.Y(), v1.Z() - v2.Z());  
}
 
Vector3d operator*(const double c, const Vector3d& v)
{
  return Vector3d(v.X()*c, v.Y()*c, v.Z()*c);
}
 
Vector3d operator*(const Vector3d& v, const double c)
{
  return Vector3d(v.X()*c, v.Y()*c, v.Z()*c);
}
 
Vector3d operator/(const Vector3d& v, const double c)
{
  return Vector3d(v.X()/c, v.Y()/c, v.Z()/c);
}

Vector3d& Vector3d::
operator+=(const Vector3d& v)
{
  // Add vector to this 
  x += v.X();
  y += v.Y();
  z += v.Z();
  return *this;
}

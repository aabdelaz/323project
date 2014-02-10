#ifndef VECTOR3D
#define VECTOR3D

class Vector3d {
 public:
  Vector3d(double x, double y, double z);
  Vector3d (): x(0), y(0), z(0) {}

  double X(void) const;
  double Y(void) const;
  double Z(void) const;
    
  void setCoords(double a, double b, double c);
    
  double dot(const Vector3d& v);
  double length();
  double lsquared();
  
  Vector3d& operator+=(const Vector3d& v);
  
  friend Vector3d operator+(const Vector3d& v1, const Vector3d& v2);
  friend Vector3d operator-(const Vector3d& v1, const Vector3d& v2);
  friend Vector3d operator*(const double c, const Vector3d& v);
  friend Vector3d operator*(const Vector3d& v, const double c);
  friend Vector3d operator/(const Vector3d& v, const double c);
  
 private:
  double x;
  double y;
  double z;

};

inline double Vector3d::
X(void) const
{
  return x;
}

inline double Vector3d::
Y(void) const
{
  return y;
}

inline double Vector3d::
Z (void) const
{
  return z;
}

inline void Vector3d::
setCoords(double newX, double newY, double newZ) 
{
  x = newX;
  y = newY;
  z = newZ;
}

#endif


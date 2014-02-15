/*
 * FILE: Ray.hh
 * DESCRIPTION:
 * Based on provided file.
 * Definition and implementation of class Ray.
 */

#ifndef RAY_HH_
#define RAY_HH_
#include "defs.hh"

/* class Ray.
 * The light ray, traversed between the objects to evaluate the shading.
 * The ray defined parametrically by the origin point O
 * and the normalized direction vector D.
 */
class Ray
{
public:
  typedef enum {
    RAY_RECURSIVE, //direct or recursive (regular) ray
    RAY_SHADOW, //shadow ray only for occlusions estimation
    RAY_INTERNAL //internal ray to estimate exit point of refracted light
  } Type;

  Type _type; //the type of the ray (used to omit unneeded computations)

  Ray()
  : _type(RAY_RECURSIVE)
  {
  }

  Ray(const Point3d &O, const Vector3d &D)
    : _type(RAY_RECURSIVE), _O(O), _D(D.normalized())
  {
  }

  //set ray origin
  Point3d& O()
  {
    return _O;
  }

  //get ray origin
  const Point3d& O() const
  {
    return _O;
  }

  //set ray direction (should be noramlized vector)
  Vector3d& D()
  {
    return _D;
  }

  //get ray direction
  const Vector3d& D() const
  {
    return _D;
  }

  /* Evaluate ray on parameter t. */
  Point3d operator()(double t) const
  {
    return _O + _D * t;
  }

private:

  Point3d _O; //Ray origin
  Vector3d _D; //Ray direction - should be a unit vector
}; //class Ray

#endif // RAY_HH_

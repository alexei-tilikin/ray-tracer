/*
 * FILE: lights_inline.hh
 * DESCRIPTION:
 * Implementation of inline functions, declared in lights.hh.
 * Resolving includes loop by moving those in separate header file.
 */

#ifndef LIGHTS_INLINE_HH_
#define LIGHTS_INLINE_HH_

#include "scene.hh"
#include "lights.hh"
#include "ray.hh"

inline bool
SphereLight::sample(const Point3d &P, const Point3d &currSample,
                    const Vector3d *base, Scene &scene, Ray &ray)
{
  //the point on the actual hemisphere is a linear combination of base vectors
  Point3d destP = _position + base[0] * currSample[0]
                            + base[1] * currSample[1]
                            + base[2] * currSample[2];
  //the direction from P to the sample on the hemisphere
  ray.D() = destP - P;

  double t, maxT = ray.D().length(); //maximal hit parameter
  ray.D() /= maxT; //normalize the direction

  return (scene.intersect_ray(ray, t) && t <= maxT);
}

#endif // LIGHTS_INLINE_HH_

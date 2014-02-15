/*
 * FILE: BoundingSphere.hh
 * DESCRIPTION:
 * Definition & implementation of class BoundingSphere.
 */

#ifndef BOUNDING_SPHERE_HH_
#define BOUNDING_SPHERE_HH_

#include <cmath>

#include "defs.hh"

/* class BoundingSphere.
 * Defines spherical bounding volume around certain point with given radius.
 * Provides intersection with ray, where two parameters of the ray returned.
 * Used as base to Sphere class and as independent bounding volume object.
 * Can be safely cloned.
 */
class BoundingSphere
{
public:

  /* Empty ctor to allocate memory for the object*/
  BoundingSphere()
  {
  }

  /* Create new bounding sphere around point c with radius r.*/
  BoundingSphere(Point3d c, double r)
    : _c(c), _r(r)
  {
  }

  /* Sphere - ray intersection test.
   * Returns the number of intersection points, which may be
   *    0 - no intersection;
   *    1 - only one intersection point
   *        (useful when sending refracted ray from inside the sphere);
   *    2 - two intersection points. This one is the normal condition for the
   *        bounding sphere.
   * Assigns t1, t2 to be the ray parameters in intersection points.
   * t1 is always the minimal parameter (the closest intersection).
   */
  int intersect(Ray& ray, double& t1, double& t2);

protected:
  Point3d _c; //sphere center point
  double _r; //sphere radius

}; //class BoundingSphere

/*########################Inline functions###################################*/

inline int
BoundingSphere::intersect(Ray& ray, double& t1, double& t2)
{
  /* Solving square equation as described in lecture slides.
   * Since b was originally multiplied by 2, we can omit this because:
   * [-b +- sqrt(b^2 - 4ac)] / 2a == [-b/2 +- sqrt(b^2/4 - ac)] / a
   * a == (rayD dot rayD) == 1 here.
   */

  const Point3d o_c = ray.O() - _c;
  const double b = (o_c | ray.D());
  const double c = (o_c | o_c) - sqr(_r);

  double d = sqr(b) - c;

  if (d < 0.0)
  { //no intersections
    return 0;
  }

  d = sqrt(d);
  //t1 <= t2 are two intersection parameters.
  t1 = -b - d;
  t2 = -b + d;

  //count the number of intersections
  int ret = 2;
  if (t1 < EPS)
  {
    --ret;
    t1 = t2;
  }
  if (t2 < EPS) --ret;

  return ret;
}

#endif // BOUNDING_SPHERE_HH_

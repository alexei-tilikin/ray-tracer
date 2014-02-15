/*
 * FILE: sphere.hh
 * DESCRIPTION:
 * Based on provided file.
 * Definition of class Sphere.
 */

#ifndef SPHERE_HH_
#define SPHERE_HH_

#include "defs.hh"
#include "object.hh"
#include "BoundingSphere.hh"


/* class Sphere.
 * Defines 3D sphere parametrically.
 * Provides intersection test with ray,
 * and spherical texture mapping
 */
class Sphere : public Object, public BoundingSphere
{
 public:

  /* New sphere.
   * c - the center point.
   * r - the radius.
   */
  Sphere(Point3d c, double r)
    :BoundingSphere(c, r)
  {
  }

  /* Ray - Sphere intersection.
   * Implements virtual object::intersect(..).
   */
  virtual int intersect(Ray& ray, double& t, Point3d& P, Vector3d& N);

  /* Spherical texture mapping.
   * Implements virtual object::texture_diffuse(..).
   */
  virtual Color3d texture_diffuse(const Point3d &P);

  virtual ~Sphere()
  {
  }
}; //class Sphere

#endif // SPHERE_HH_

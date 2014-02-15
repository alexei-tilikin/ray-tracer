/*
 * FILE: sphere.cc
 * DESCRIPTION:
 * Implementation of class Sphere.
 */
#include <cmath>

#include "sphere.hh"

int Sphere::intersect(Ray& ray, double& t, Point3d& P, Vector3d& N)
{
  //using the BoundingSphere to find the intersection

  double t1, t2;
  if (BoundingSphere::intersect(ray, t1, t2) == 0)
  { //no intersection
    return 0;
  }

  //find the closest intersection
  t = t1;
  assert(t > EPS);
  //the intersection point
  P = ray(t);
  //the normal on sphere: vector from center point to intersection point
  N = (P - _c).normalize();

  return 1;
}


Color3d Sphere::texture_diffuse(const Point3d &P)
{ //implementaion spherical texture mapping

  //no texture map or zero-radius sphere
  if (_diffuse_texture == NULL || _r < EPS) return COLOR_WHITE;

  const Point3d cartesian = P - _c;
  double norm2D = sqrt(sqr(cartesian[0]) + sqr(cartesian[2]));

  /*compute spherical coordinates (as defined by spherical mapping):
   * u = [atan(x/z) + pi] / 2pi
   * v = [atan(y/sqrt(x^2+z^2)) + pi/2] / pi
   */

  //computing the angles
  TexCoord pos(atan2(cartesian[0], cartesian[2]), atan2(cartesian[1], norm2D));

  assert(pos[0] >= -M_PI && pos[0] <= M_PI);
  assert(pos[1] >= -HALF_PI && pos[1] <= HALF_PI);

  //translating into [0, 1] range for texture coordinates
  pos[1] *= INV_PI; //divide by PI
  pos[1] += 0.5;

  pos[0] *= INV_TWO_PI; //divide by 2*PI
  pos[0] += 0.5;

  return map_texture(pos);
}

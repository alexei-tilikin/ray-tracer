/*
 * FILE: toolkits.hh
 * DESCRIPTION:
 * Defines toolkit functions for some primitives in 3D space:
 * plane, triangle.
 * Using C-style functions to avoid overhead on creating class instances.
 */

#ifndef TOOLKITS_HH_
#define TOOLKITS_HH_

#include "defs.hh"
#include "object.hh"
#include "print_debug.hh"

/*########################Plane toolkit#######################################*/
namespace plane
{

/* The pointer object for plane.
 * This one only references to the plane parameters.
 */
struct Plane
{
  const Vector3d *N; //the normal
  const Point3f *Q; //any point on a plane
};

/* Ray - Plane intersection.
 * N - the normal to the plane.
 * Q - any point on the plane.
 * ray - the ray to intersect with.
 * t - the ray parameter in case of intersection.
 * P - the intersection point.
 * Returns true iff the ray intersects with the plane (t>0.0).
 */
inline bool
intersect(const Vector3d& N, const Point3d &Q, const Ray& ray,
          double& t, Point3d &P)
{
  double NdotD = ray.D() | N;

  if (NdotD == 0.0)
  { //ray is parallel to the plane
    return false;
  }

  double NdotQO = N | (Q - ray.O());

  /* The ray may intersect with the plane in point ray_O + t*ray_D.
   * Solving linear system:
   * (ray_O + t*ray_D - Q) dot N == 0
   * That is the vector {ray(t),  Q} should lie on the plane
   * and be in 90 degrees with the normal.
   * Finding t:
   * t = [N dot (Q - ray_O)] / (ray_D dot N)
   */
  t = NdotQO / NdotD;

  if (t < EPS)
  { //intersection before the ray origin
    return false;
  }

  P = ray(t);
  return true;
}

/* Wrapping function to take the plane parameters from struct Plane.*/
inline bool
intersect(const Plane& plane, const Ray& ray, double& t, Point3d& P)
{
  return intersect(*plane.N, Point3d(*plane.Q), ray, t, P);
}

} //namespace plane

/*########################Triangle toolkit###################################*/
namespace triangle
{

/* Check if the given point lays inside the triangle.
 * p - the point to test whether it's inside the triangle.
 * tr0, tr1, tr2 - triangle vertices.
 * barycentric - the barycentric coordinates of the point p for current triangle.
 */
inline bool
fallsIn(const Point3d& p, const Point3d& tr0, const Point3d& tr1,
        const Point3d& tr2, Point2d &barycentric)
{
  /* Computing needed vectors for area estimations.
   * The process was described in the class.
   * Lengths of edges estimate areas of internal triangles.
   * Fractions of the areas estimate the barycentric coordinates.
   */
  const Vector3d tr0_1(tr1 - tr0);
  const Vector3d tr0_2(tr2 - tr0);
  const Vector3d tr0_p(p - tr0);
  const Vector3d p_tr1(tr1 - p);
  const Vector3d p_tr2(tr2 - p);

  //area factor for the triangle
  const double tr_area = (tr0_1 % tr0_2).sqrnorm();
  assert(tr_area > 0.0);

  barycentric[0] = sqrt((p_tr1 % p_tr2).sqrnorm() / tr_area);
  //inside test: positive barycentric coordinate
  if (barycentric[0] < 0.0 || barycentric[0] > 1.0) return false;

  barycentric[1] = sqrt((tr0_p % tr0_2).sqrnorm() / tr_area);
  if (barycentric[1] < 0.0 || barycentric[0] + barycentric[1] > 1.0)
  { //inside test: positive barycentric coordinate, sum at most 1
    return false;
  }

  double b2 = sqrt((tr0_1 % tr0_p).sqrnorm() / tr_area);
  if (b2 < 0.0 || !approxEqual(barycentric[0] + barycentric[1] + b2, 1.0))
  { //inside test: sum of all 3 coordinates must be 1
    return false;
  }

  return true; //all tests passed: the point is inside
}

/* Interpolates texture coordinates from triangle vertices (c1, c2, c3)
 * to get texture coordinate for barycentric point.
 * coordType may be either scalar, Point2d or Point3d.
 */
template <class CoordType>
inline CoordType
interpolateBarycentric(const Point2d &barycentric, const CoordType &c0,
                       const CoordType &c1, const CoordType &c2)
{
  return c0 * barycentric[0] + c1 * barycentric[1]
         + c2 * (1.0 - barycentric[0] - barycentric[1]);
}

} //namespace triangle

#endif // TOOLKITS_HH_

/*
 * FILE: Cylinder.hh
 * DESCRIPTION:
 * Definition of Cylinder class: our arbitrary parametric object.
 */

#ifndef CYLINDER_HH_
#define CYLINDER_HH_

#include "defs.hh"
#include "object.hh"
#include "toolkits.hh"

/* class Cylinder.
 * The main axis defined to always be parallel to Y.
 * Cylinder can be defined by the center point of the bottom cap,
 * the radius and the height.
 * Texture mapping uses cylindrical coordinates.
 */
class Cylinder: public Object
{
public:
  //normals for cap disks
  static const Vector3d NORMAL_TOP_CAP;
  static const Vector3d NORMAL_BOTTOM_CAP;

  /* The constructor.
   * baseCenter - center point of lower base cap.
   * radius - the radius along all the cylinder.
   * h - the distance between lower and higher caps.
   */
  Cylinder(const Point3d &baseCenter = Point3d(0.0),
           double radius = 1.0, double height = 1.0)
    : _bottomCenter(baseCenter), _topCenter(baseCenter),
      _radius(radius), _sqrRadius(sqr(radius)), _invRadius(1.0 / radius),
      _invHeight(1.0 / height)
  {
    _topCenter[1] += height;
  }

  /* Ray - Cylinder intersection.
   * Implements virtual object::intersect(..).
   */
  virtual int intersect(Ray& ray, double& t, Point3d& P, Vector3d& N);

  /* Cylindrical texture mapping.
   * Implements virtual object::texture_diffuse(..).
   */
  virtual Color3d texture_diffuse(const Point3d &P);

  virtual ~Cylinder()
  {
  }

protected:
  Point3d _bottomCenter; //base center with minimal Y coordinate
  Point3d _topCenter; //base center with maximal Y coordinate
  double _radius; //the cylinder radius
  //common used values for faster calculations
  double _sqrRadius; //square of radius
  double _invRadius; //(1/radius) for fast normal normalization
  double _invHeight; //(1/height) for fast computation of texture coordinates

  /* Special case of intersection when the ray is parallel to Y axis. */
  bool parallelRay(Ray& ray, double& t, Point3d& P, Vector3d& N);

}; //class Cylinder

inline bool
Cylinder::parallelRay(Ray& ray, double& t, Point3d& P, Vector3d& N)
{
  if (ray.O()[0] > _bottomCenter[0] + _radius
      || ray.O()[0] < _bottomCenter[0] - _radius
      || ray.O()[2] > _bottomCenter[2] + _radius
      || ray.O()[2] < _bottomCenter[2] - _radius)
  { //the ray has constant X,Z coordinates and they're out of the cylinder range
    return false;
  }

  //testing only intersection with caps
  t = INF;
  double lastT;
  if (plane::intersect(NORMAL_BOTTOM_CAP, _bottomCenter, ray, lastT, P))
  {
    t = lastT;
    N = NORMAL_BOTTOM_CAP;
  }
  if (plane::intersect(NORMAL_TOP_CAP, _topCenter, ray, lastT, P) && lastT < t)
  {
    t = lastT;
    N = NORMAL_TOP_CAP;
  }

  return (t != INF);
}

#endif // CYLINDER_HH_

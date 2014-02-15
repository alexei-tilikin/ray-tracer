/*
 * FILE: BoundingBox.hh
 * DESCRIPTION:
 * Definition of class BoundingBox.
 */

#ifndef BOUNDING_BOX_HH_
#define BOUNDING_BOX_HH_
#include "defs.hh"
#include "toolkits.hh"

/* class BoundingBox.
 * Constructed from two opposite corners.
 * Can be safely cloned.
 * Provides intersection with ray test.
 */
class BoundingBox
{
public:

  /*Pre-defined normals for canonical directions.*/
  static const Vector3d NORMAL_LEFT;
  static const Vector3d NORMAL_RIGHT;
  static const Vector3d NORMAL_UP;
  static const Vector3d NORMAL_DOWN;
  static const Vector3d NORMAL_FRONT;
  static const Vector3d NORMAL_BACK;

  /* Empty ctor for memory preallocation.*/
  BoundingBox()
  {
  }

  /* Copy ctor.*/
  BoundingBox(const BoundingBox &rhs)
    : _minCorner(rhs._minCorner), _maxCorner(rhs._maxCorner)
  {
    setSides();
  }

  /*Assignment operator.*/
  BoundingBox &operator=(const BoundingBox &rhs)
  {
    if (this == &rhs) return *this;
    _minCorner = rhs._minCorner;
    _maxCorner = rhs._maxCorner;
    setSides();
    return *this;
  }

  /* BoundingBox defined  by two opposite corners:
   * minCorner - the corner with all 3 coordinates minimal.
   * maxCorner - the corner with all 3 coordinates maximal.
   */
  BoundingBox(const Point3f &minCorner, const Point3f &maxCorner);

  /* Box - ray intersection.
   * Returns:
   * false - if no intersection
   * true - if intersection at least in one point detected.
   */
  bool intersect(const Ray& ray);

  /* Access the maximal coordinates of the box.*/
  const Point3f &maxBound()
  {
    return _maxCorner;
  }

  /* Access the minimal coordinates of the box.*/
  const Point3f &minBound()
  {
    return _minCorner;
  }


protected:
  plane::Plane _sides[6];
  Point3f _minCorner;
  Point3f _maxCorner;

  /*Set planes for each box side.*/
  void setSides();

  /*Boundary test for intersection point candidate.*/
  bool fallsIn(const Point3f& P) const;

}; //class BoundingBox

/*########################Inline functions###################################*/

inline bool BoundingBox::fallsIn(const Point3f& P) const
{
  if (P[0] + EPS < _minCorner[0] || P[0] - EPS > _maxCorner[0]
     || P[1] + EPS < _minCorner[1] || P[1] - EPS > _maxCorner[1]
     || P[2] + EPS < _minCorner[2] || P[2] - EPS > _maxCorner[2])
  { //intersection out of borders
    return false;
  }
  return true;
}

inline bool BoundingBox::intersect(const Ray& ray)
{
  Ray r = ray;
  Point3d P;
  double t;

  //check each of 6 planes for intersection
  for (uint i = 0; i < 6; ++i)
  {
    if (plane::intersect(_sides[i], r, t, P) == 1 && fallsIn(Point3f(P)))
    {
      return true;
    }
  }

  return false; //zero intersections
}

#endif // BOUNDING_BOX_HH_

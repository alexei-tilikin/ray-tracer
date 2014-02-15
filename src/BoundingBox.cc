/*
 * FILE: BoundingBox.cc
 * DESCRIPTION:
 * Implementation of class BoundingBox.
 */
#include "BoundingBox.hh"

const Vector3d BoundingBox::NORMAL_LEFT(-1.0, 0.0, 0.0);
const Vector3d BoundingBox::NORMAL_RIGHT(1.0, 0.0, 0.0);
const Vector3d BoundingBox::NORMAL_UP(0.0, 1.0, 0.0);
const Vector3d BoundingBox::NORMAL_DOWN(0.0, -1.0, 0.0);
const Vector3d BoundingBox::NORMAL_FRONT(0.0, 0.0, 1.0);
const Vector3d BoundingBox::NORMAL_BACK(0.0, 0.0, -1.0);

BoundingBox::BoundingBox(const Point3f &minCorner, const Point3f &maxCorner)
  : _minCorner(minCorner), _maxCorner(maxCorner)
{
  //assuming non-zero box volume
  assert((_maxCorner - _minCorner).length() > EPS);

  setSides();
}

void BoundingBox::setSides()
{
  //left side
  _sides[0].N = &NORMAL_LEFT;
  _sides[0].Q = &_minCorner;
  //right side
  _sides[1].N = &NORMAL_RIGHT;
  _sides[1].Q = &_maxCorner;

  //up side
  _sides[2].N = &NORMAL_UP;
  _sides[2].Q = &_maxCorner;
  //bottom side
  _sides[3].N = &NORMAL_DOWN;
  _sides[3].Q = &_minCorner;

  //near side
  _sides[4].N = &NORMAL_FRONT;
  _sides[4].Q = &_maxCorner;
  //far side
  _sides[5].N = &NORMAL_BACK;
  _sides[5].Q = &_minCorner;
}

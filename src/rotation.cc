/*
 * FILE: rotation.cc
 * DESCRIPTION:
 * Implementation of class Rotation.
 */
#include "rotation.hh"

//The canonical axis
const Vector3d Rotation::CANONICAL_AXIS[] = {
  Vector3d(1.0, 0.0, 0.0),
  Vector3d(0.0, 1.0, 0.0),
  Vector3d(0.0, 0.0, 1.0)
};

const Vector3d Rotation::ZERO_VECTOR(0.0, 0.0, 0.0);

void Rotation::findRotation(const Vector3d &from, const Vector3d &to)
{
  //find the main rotation angle
  _angleCos = from | to;
  if (approxEqual(_angleCos, 1.0))
  { //empty rotation
    return;
  }
  else if (approxEqual(_angleCos, -1.0))
  { //reflection: PI radians
    _invert = true;
    _reflection = Vector3d(1.0, 1.0, 1.0);
    //detect which coordinates should be inverted in this reflection case.
    for (uint i = 0; i < 3; ++i)
    {
      if (to[i] != 0.0) _reflection[i] = -1.0;
    }
    return;
  }

  //else: general rotation
  _empty = false;
  _angleSin = cos2sin(_angleCos);

  //find the rotation axis
  Vector3d axis = from % to;
  //compute all the data about the rotation
  prepare(axis.normalize());

  //testing rotation and deducing the angle sign
  Vector3d test = from;
  rotate(test);

  if (!approxEqual(test | to, 1.0))
  { //the angle assumed to be negative
    _invert = true;
  }

  //final test
  DEBUG_EXEC(
   test = from;
   rotate(test);
   double dotProd = test | to;
   if (!approxEqual(dotProd, 1.0))
   {
     cerr << "\t\tRotation error! Dot product: " << dotProd << endl;
     assert(0);
   }
  )
}

void Rotation::prepare(const Vector3d &axis)
{ //axis assumed to be normalized

  //detecting canonical rotation
  Vector3d axis_abs = vec_abs(axis);
  for (uint i = 0; i < 3; ++i)
  {
    if (approxEqual(axis_abs, (const Point3d &)CANONICAL_AXIS[i]))
    { //found the canonical axis for the rotation
      _axis = i;
      break;
    }
  }

  if (_axis == -1) _canonical = false;
  else return;

  //invalidate projections
  _projAxis[0] = -1;
  _projAxis[1] = -1;

  //projections of rotation axis: previous stage and current stage
  Vector3d prev = axis, curr;

  /* finding two projection rotations for the axis
   * i - current projection index: {0, 1}
   * dim - current candidate dimension for projection: {0, 1, 2}
   */
  for (int i = 0, dim = 0; i < 2 && dim < 3; ++dim)
  {
    curr = prev;
    curr[dim] = 0.0; //projecting the axis
    if (approxEqual(curr, ZERO_VECTOR))
    { //bad projection, skipping
      continue;
    }

    curr.normalize();

    //determine appropriate rotation axis for current projection
    //this generic code tested for all 3 possible cases
    if (approxEqual(vec_abs(curr),
                    (const Point3d &)CANONICAL_AXIS[addMod(dim, uint, 1, 3)]))
    { //special case: projected axis becomes canonical
      _projAxis[i] = addMod(dim, int, 2, 3);
    }
    else
    {
      _projAxis[i] = addMod(dim, int, 1, 3);
    }

    //compute the angle
    _projCos[i] = prev | curr;
    _projSin[i] = cos2sin(_projCos[i]);
    _projInvert[i] = false;

    //performing rotation to verify the angle sign
    Vector3d test = prev;
    rotateCanonical(test, _projCos[i], _projSin[i], _projAxis[i], _projInvert[i]);
    if (!approxEqual(test | curr, 1.0))
    { //the angle assumed to be negative
      _projInvert[i] = true;
    }

    //prepare for next projection
    prev = curr;
    ++i;
  }

  //setting the main rotation axis after projections
  //curr holds final projection of rotation axis onto canonical direction
  curr = vec_abs(curr);

  //find which canonical direction the axis meets
  for (uint i = 0; i < 3; ++i)
  {
    if (approxEqual(curr, (const Point3d &) CANONICAL_AXIS[i]))
    {
      _axis = i;
      break;
    }
  }
  assert(_axis != -1); //this time it must meet some canonical direction!
}

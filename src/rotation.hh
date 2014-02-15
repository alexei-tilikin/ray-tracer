/*
 * FILE: rotation.hh
 * DESCRIPTION:
 * Tools for vector 3D rotation.
 * Definition of Rotation class.
 */

#ifndef ROTATION_HH_
#define ROTATION_HH_

#include <cstring>

#include "defs.hh"
#include "print_debug.hh"

/* class Rotation.
 * Provides tools for 3D rotation of vectors.
 * The rotation can be defined either by the axis and the angle,
 * or by two 3D vectors of source and destination.
 */
class Rotation
{
public:
  //Canonical axis definitions
  static const Vector3d CANONICAL_AXIS[3];
  static const Vector3d ZERO_VECTOR;

  /*Empty ctor for memory allocation.*/
  Rotation()
  {
  }

  /* Define rotation between _from_ and _to_ vectors.*/
  Rotation(const Vector3d &from, const Vector3d &to)
    : _empty(true), _invert(false), _canonical(true), _axis(-1)
  {
    findRotation(from.normalized(), to.normalized());
  }

  /* Define rotation around the given axis by the given angle.
   * Here the angle restricted to be in the range [-PI/2, PI/2].
   */
  Rotation(const Vector3d &axis, double angle);

  /* Assignment operator for sane instance copying.*/
  Rotation &operator=(const Rotation &rhs);

  /* Apply the rotation on the given vector.
   * The input vector modified - rotated in 3D space.
   */
  void rotate(Vector3d &vec);

  /* Returns true iff current rotation is by a zero-angle.*/
  bool isEmpty()
  {
    return (_empty && !_invert);
  }

private:
  /*data about main rotation angle*/
  bool _empty; //empty rotation may be either 0 or PI radians.
  bool _invert; //information about angle sign (true for negative angles)
  bool _canonical; //true when rotation axis initially canonical (simple case)
  int _axis; //canonical axis around which the main rotation occurs
  double _angleCos; //cosine of the main rotation angle
  double _angleSin; //sine of the main rotation angle
  Vector3d _reflection; //reflection dimensions (only for PI rotations)

  /*two projections of rotation axis to meet canonical direction*/
  //rotation angles for projective rotations
  double _projCos[2];
  double _projSin[2];
  int _projAxis[2]; //canonical axis to perform the projection onto
  uchar _projInvert[2]; //negative angle indicators

  /* Estimates appropriate rotation for vector _from_
   * to be rotated to match with _to_.
   * Assumes _from_ and _to_ are normalized vectors.
   */
  void findRotation(const Vector3d &from, const Vector3d &to);

  /* Prepares rotation data around the given axis.
   * Assumes axis vector is normalized.
   */
  void prepare(const Vector3d &axis);

}; //class Rotation

/*########################Inline functions###################################*/

inline Rotation::Rotation(const Vector3d &axis, double angle)
  : _empty(true), _invert(false), _canonical(true), _axis(-1)
{
  assert(!(angle < -HALF_PI || angle > HALF_PI));

  if (!approxEqual(angle, 0.0))
  { //skip empty rotation
    _empty = false;
    _angleCos = cos(angle);
    _angleSin = cos2sin(_angleCos);
    _invert = (angle < 0.0);
    prepare(axis.normalized());
  }
}

inline Rotation & Rotation::operator=(const Rotation &rhs)
{
  if (this == &rhs) return *this;

  _empty = rhs._empty;
  _invert = rhs._invert;
  _canonical = rhs._canonical;
  _axis = rhs._axis;
  _angleCos = rhs._angleCos;
  _angleSin = rhs._angleSin;
  _reflection = rhs._reflection;

  memcpy(_projCos, rhs._projCos, sizeof(double) * 2);
  memcpy(_projSin, rhs._projSin, sizeof(double) * 2);
  memcpy(_projAxis, rhs._projAxis, sizeof(int) * 2);
  memcpy(_projInvert, rhs._projInvert, 2);

  return *this;
}

/* Rotate vector around the chosen canonical axis.
 * Input:
 * v - the vector to be rotated - modified.
 * angleCos, angleSin - cosine and sine of rotation angle.
 * axis - one of {0,1,2} for X, Y or Z rotation axis.
 * inverted - if true, then angle sign inverted.
 */
inline void
rotateCanonical(Vector3d &v, double angleCos, double angleSin, int axis,
                bool inverted)
{
  if (axis < 0) return;
  assert(axis < 3);
  if (approxEqual(angleCos, 1.0))
  { //zero angle
    return;
  }

  //inverted rotation
  if (inverted) angleSin = -angleSin;
  //keep the original vector
  const Vector3d orig(v);

  //next coordinate after the chosen axis
  const uint axis1 = addMod(axis, uint, 1, 3);
  //next-next coordinate after the chosen axis
  const uint axis2 = addMod(axis1, uint, 1, 3);

  //using the cyclic property of rotation matrices around canonical axis
  v[axis1] = angleCos * orig[axis1] + angleSin * orig[axis2];
  v[axis2] = -angleSin * orig[axis1] + angleCos * orig[axis2];
}

inline void Rotation::rotate(Vector3d &vec)
{
  if (_empty && !_invert)
  { //zero-rotation
    return;
  }
  if (_empty && _invert)
  { //reflection
    vec *= _reflection; //reflect only the chosen coordinates
    return;
  }
  if (_canonical)
  { //canonical rotation
    rotateCanonical(vec, _angleCos, _angleSin, _axis, _invert);
    return;
  }

  //else: general rotation

  //projection rotations to set the axis in canonical direction
  rotateCanonical(vec, _projCos[0], _projSin[0], _projAxis[0], _projInvert[0]);
  rotateCanonical(vec, _projCos[1], _projSin[1], _projAxis[1], _projInvert[1]);

  //the main rotation
  rotateCanonical(vec, _angleCos, _angleSin, _axis, _invert);

  //backward rotations to disable the axis projections
  rotateCanonical(vec, _projCos[1], _projSin[1], _projAxis[1], !_projInvert[1]);
  rotateCanonical(vec, _projCos[0], _projSin[0], _projAxis[0], !_projInvert[0]);
}

#endif // ROTATION_HH_

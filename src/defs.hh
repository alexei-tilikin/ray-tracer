/*
 * FILE: defs.hh
 * DESCRIPTION:
 * Defines type definitions for all project classes.
 * Provides global functions of common purpose.
 */

#ifndef DEFS_HH_
#define DEFS_HH_

#include <cmath>
#include <string>

#include <EasyBMP.h>

#include "MyVecs.hh"
#include "print_debug.hh"

//unsigned types
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;

//boundary floating-point values
#define INF 1e30
#define EPS 1e-6

//trigonometric constants
#define TWO_PI (M_PI * 2.0)
#define HALF_PI (M_PI * 0.5)
#define INV_PI (1.0 / M_PI)
#define INV_TWO_PI (1.0 / TWO_PI)

//boundary states of points
#define EPS_POINT Point3d(EPS, EPS, EPS)
#define INF_POINT Point3d(INF, INF, INF)

//corner values for colors
#define COLOR_WHITE Color3d(1.0, 1.0, 1.0)
#define COLOR_BLACK Color3d(0.0, 0.0, 0.0)

//recursion constants
#define VIS_THRES (0.05) //ray energy threshold

/* Default multiplicative factor for vis on each recursion level.
 * The maximal recursion depth can be 6 when energy threshold is 0.05
 */
#define VIS_FACTOR (0.65)

/* Calculates (val + add) MOD mod.
 * Result casted to val_type
 */
#define addMod(val, res_type, add, mod) \
  ( res_type((uint(val) + uint(add)) % uint(mod)) )


/* Generic square function.
 * For vectors, performs element-wise multiplication of vector with itself.
 */
template<class T>
inline T sqr(const T &in)
{
  return in * in;
}

/* Returns true iff a approximately equals b.*/
inline bool approxEqual(double a, double b)
{
  return (fabs(a - b) < EPS);
}

/* Returns true iff p1 approximately the same point as p2.*/
inline bool approxEqual(const Point3d &p1, const Point3d &p2)
{
  return (approxEqual(p1[0], p2[0])
          && approxEqual(p1[1], p2[1])
          && approxEqual(p1[2], p2[2]));
}

/* Returns absolute direction of vector:
 * all coordinates set to be non-negative.
 * Useful for direction-insensitive vectors comparison.
 */
inline Vector3d vec_abs(const Vector3d &vec)
{
  return Vector3d(fabs(vec[0]), fabs(vec[1]), fabs(vec[2]));
}

/* Returns vis factor of given RGB color.
 * Returned vis factor will always be bound by VIS_FACTOR,
 * but may be lesser for low intensity colors.
 */
inline double colorEnergy(const Color3d &c)
{
  //accurate luminance estimation
  double energy = (c[0] * 0.3 + c[1] * 0.59 + c[2] * 0.11);
  return std::min(energy, VIS_FACTOR);
}

/* Converts cosine to sine.
 * Not preserves the sign: returns absolute value of sine.
 */
inline double cos2sin(double cos_val)
{
  if (cos_val == 1.0) return 0.0;
  return sqrt(1.0 - sqr(cos_val));
}

/* Converts degrees to radians.*/
inline double deg2rad(double degrees)
{
  static const double conversion = M_PI / 180.0;
  return degrees * conversion;
}

/* Loads bmp image from the given path into BMP object .
 * Triggers process termination if image not loaded.
 * Helps to avoid annoying SEGFAULT as result of missing texture files.
 */
inline BMP * loadBMP(const std::string &filename)
{
  BMP *img = new (std::nothrow) BMP();
  if (img == NULL || !img->ReadFromFile(filename.c_str()))
  {
    cerr << "Couldn't load image from file: " << filename << endl;
    delete img;
    exit(RC_INPUT_ERROR);
  }
  return img;
}

#endif // DEFS_HH_

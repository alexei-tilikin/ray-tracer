/*
 * FILE: lights.hh
 * DESCRIPTION:
 * Based on provided file.
 * Defines classes for all types of lights.
 */

#ifndef LIGHTS_HH_
#define LIGHTS_HH_

#include <vector>

#include "defs.hh"

//pre-declarations
class Scene;
class Ray;

/* Base class for all Light types.
 * Its an abstract light without direction and position.
 * Used for ambient light.
 */
class Light
{
public:
  Light(const Color3d &color)
    : _color(color)
  {
  }

  /*Access to the color field.*/
  Color3d &color()
  {
    return _color;
  }

  /* Returns light color specifically for point P.
   * May use the current scene instance to send shadow rays.
   * Returns the color after shadow estimation in the current point.
   * Depends on the light type.
   */
  virtual Color3d color(const Point3d &P, Scene &scene)
  {
    return _color;
  }

  virtual ~Light()
  {
  }

protected:
  Color3d _color; //the color of the light source
}; //class Light

typedef Light AmbientLight;

/* LocatedLight has defined position point.
 * Used for point light sources
 * and as a base of any other position-defined lights.
 */
class LocatedLight : public Light
{
public:
  LocatedLight(const Point3d &position = Point3d(0, 0, 0),
             const Color3d &color = Color3d(0, 0, 0))
    : Light(color), _position(position)
  {
  }

  const Point3d &position() const
  {
    return _position;
  }

  virtual Color3d color(const Point3d &P, Scene &scene);

  virtual ~LocatedLight()
  {
  }

protected:
  Point3d _position;
};

typedef LocatedLight PointLight;

/* SphereLight has a shape of sphere with real volume.
 * Defined by the sphere center point and the radius.
 */
class SphereLight : public LocatedLight
{
public:

/*
 * densityFactor - defines how many levels to sample from the hemisphere.
 *      On each level, the number of sample points will be 2^densityFactor.
 *      One additional sample will be at the hemisphere peak point.
 */
  SphereLight(const Point3d &center = Point3d(0, 0, 0),
              double radius = 1.0,
              const Color3d &color = Color3d(0, 0, 0),
              uint numLayers = 0);

  /* Estimates intensity of light,
   * what arrives from current light sphere to point P.
   */
  virtual Color3d color(const Point3d &P, Scene &scene);

  virtual ~SphereLight()
  {
  }

protected:
  //array of sample points with their layer identifiers
  typedef std::vector< std::pair<uint, Point3d> > SamplesArray;

  double _radius;
  //number of layers for the light estimation. See explanation in README.
  uint _nLayers;
  double _normFactor; //normalization factor for sum of intensity from all samples
  //pre-computed canonical sample points on the light sphere
  SamplesArray _samples;

  /* Subroutine for color(..).
   * Sends shadow ray to perform occlusion test for certain sample point.
   * Returns true iff the sample point occluded (i.e. drops shadow).
   */
  bool sample(const Point3d &P, const Point3d &currSample,
              const Vector3d *base, Scene &scene, Ray &ray);
}; //class SphereLight

#endif // LIGHTS_HH_

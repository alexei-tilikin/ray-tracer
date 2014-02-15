/*
 * FILE: lights.cc
 * DESCRIPTION:
 * Implementation of classes: LocatedLight, SphereLight.
 */
#include <list>

#include "threading.hh"
#include "lights_inline.hh"
#include "lights.hh"
#include "scene.hh"
#include "toolkits.hh"

Color3d LocatedLight::color(const Point3d &P, Scene &scene)
{
  double t;
  double maxT = (_position - P).length();
  Ray ray(P, _position - P);
  ray._type = Ray::RAY_SHADOW;

  /* Send shadow ray from P to the light position.
   * If ray intersects before it reaches the light source,
   * then the light is occluded.
   */
  if (scene.intersect_ray(ray, t) && t <= maxT)
  {
    return COLOR_BLACK;
  }
  return _color;
}

SphereLight::SphereLight(const Point3d &center, double radius,
                         const Color3d &color, uint numLayers)
  : LocatedLight(center, color), _radius(radius), _nLayers(numLayers),
    _normFactor(0.0)
{
  typedef std::list< std::pair<uint, Point3d> > SamplesList;

  const uint tid = threadId(); //id of current (main) thread

  if (numLayers == 0)
  { //setting default number of layers
    if (_radius < 5.0) _nLayers = 2;
    else if (_radius < 50.0) _nLayers = 3;
    else _nLayers = 4;
  }

  SamplesList orderedSamples; //list of sample points

  /* Angle terms for spherical coordinates translation:
   * lo - is for longitude angle, la - the latitude angle;
   * On the hemisphere: 0 <= lo <= HALF_PI; -PI <= la <= PI;
   */
  double la = 0.0; //the latitude angle
  double cos_lo = 0.0, sin_lo = 0.0, cos_la = 0.0, sin_la = 0.0;
  double lo_step = 1.0 / double(_nLayers); //step for cosine value
  double offsetRange = 0.5 * lo_step; //range of random offset
  double la_step = 0.0; //step for angle value in radians
  double cos_lo_biased; //randomly biased longitude

  Point3d sample; //current sample point on the hemisphere
  cos_lo = lo_step;

  //iterate over all layers
  //not sampling on hemisphere border
  for (uint i = 1; i < _nLayers; ++i)
  {
    /* latitude angle step depends on the layer radius,
     * which is [_radius * cos_lo].
     * The step set to keep constant density of samples on each layer:
     * one sample per each pi/2 pixels of circle distance.
     */
    la_step = HALF_PI / (_radius * cos_lo);

    la = -M_PI; //starting position of latitude on current level

    //iterate over all sample angles for the current level
    while (la < M_PI)
    {
      //randomly biased longitude
      cos_lo_biased = cos_lo + randomf(tid, -offsetRange, 0.0);
      assert(cos_lo_biased < 1.0 && cos_lo_biased > 0.0);
      sin_lo = cos2sin(cos_lo_biased);

      //current latitude angle
      cos_la = cos(la);
      sin_la = cos2sin(cos_la);
      if (la < 0.0) sin_la = -sin_la;

      //converting to Cartesian coordinates
      sample = Point3d(cos_lo_biased * cos_la, sin_lo, cos_lo_biased * sin_la);
      sample *= _radius;

      orderedSamples.push_back(std::make_pair(i, sample));

      //accumulate the normalization factor
      _normFactor += sample[1];

      la += la_step; //next sample on current level;
    } //end of latitude loop

    cos_lo += lo_step; //increment to the next (lower) layer
  } //end of longitude loop

  //account the peak point in normalization factor
   _normFactor += _radius;

   //invert the normalization factor for faster normalization
   _normFactor = 1.0 / _normFactor;

  //allocate all needed space in vector
  _samples.reserve(orderedSamples.size());
  //copy all the samples from list to vector
  for (SamplesList::const_iterator it = orderedSamples.begin(),
                                   end = orderedSamples.end();
       it != end; ++it)
  {
    _samples.push_back(*it);
  }
  orderedSamples.clear();
}

Color3d SphereLight::color(const Point3d &P, Scene &scene)
{
  //fast skipping very low lights
  if (approxEqual(_color, EPS_POINT)) return COLOR_BLACK;

  //total intensity (the energy level) acquired from the light source
  double intensity = 0.0;

  //here storing the base vectors for the hemisphere
  Vector3d base[3];

  base[1] = P - _position; //the normal to the base plane: "new Y-axis"

  //path length from the light to the point
  double maxT = base[1].length() - _radius;

  //special case: the point is inside the light sphere
  if (maxT < EPS) return COLOR_WHITE;

  double t;

  //detecting the base plane: sending ray to point on the plane
  //rayD = _position + offset - P is the direction toward (_position + offset)
  Ray ray(P, -base[1] + Vector3d(0.5, 0.5, 0.5));
  ray._type = Ray::RAY_SHADOW;

  base[1].normalize();

  Point3d planeP(0.0);
  if (!plane::intersect(base[1], _position, ray, t, planeP))
  {
    assert(0);
  }

  //determine two orthogonal vectors on the base plane: "new X and Z axis"
  //vector from base plane center to the (0.5, 0.5, 0.5) offset
  base[0] = planeP - _position;
  base[0].normalize();
  //any orthogonal vector
  base[2] = base[0] % base[1];
  base[2].normalize();

  //accounting the peak point
  ray.D() = (_position - P).normalized(); //direction from P to the peak point
  if (!scene.intersect_ray(ray, t) || t > maxT)
  { //the ray to the peak point not occluded by other object
    intensity += _radius;
  }

  //Vector3d destP;
  Point3d currSample; //current sample point
  uint currLayer; //current layer number (identifier)

  //forward scan iterator
  SamplesArray::const_iterator it = _samples.begin(), end = _samples.end();
  //backoff iterator
  SamplesArray::const_iterator lastOccluded = _samples.begin(), backStep;

  //iterate over all pre-computed sample points
  while (it != end)
  {
    currSample = it->second;
    currLayer = it->first;

    //testing current sample point for occlusion
    if (sample(P, currSample, &base[0], scene, ray))
    {
      if (it == _samples.begin()) goto L_SamplesLoopNext; //continue;

      //go back and remove accounted samples, until meet non-occluded sample
      for (backStep = it - 1; backStep != lastOccluded; --backStep)
      {
        currSample = backStep->second;
        //testing sample point for occlusion
        if (sample(P, currSample, &base[0], scene, ray))
        { //previous sample was also occluded: removing it from total intensity
          intensity -= currSample[1];
        }
        //reached first non-occluded sample: assuming the shadow begins from here
        else
        { //not checking more backward: assuming all the rest not occluded
          lastOccluded = it;
          goto L_SamplesLoopNext; //'continue' for external while-loop
        }
      } //end of backoff loop

      lastOccluded = it;
      goto L_SamplesLoopNext; //continue;
    }

    //else - the ray not occluded. Assuming next neighbors won't be occluded
    intensity += currSample[1]; //account current sample intensity

    //skipping next samples, assuming they're also not occluded
    for (uint skip = 0; skip < uint(_radius / TWO_PI); ++skip)
    {
      if (++it == end) goto L_SamplesLoopExit; //break;
      if (it->first != currLayer)
      { //if next sample not from the same layer, continue from there
        --it;
        goto L_SamplesLoopNext; //continue;
      }
      //otherwise next sample is very close to the current:
      //assuming it's also not occluded
      intensity += (it->second)[1];
    }

    L_SamplesLoopNext: //this label allows to jump from the nested loop
    ++it;
  } L_SamplesLoopExit: //end of samples loop

  //the final color is a weighted average of all samples
  return _color * intensity * _normFactor;
}

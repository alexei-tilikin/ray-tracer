/*
 * FILE: scene.hh
 * DESCRIPTION:
 * Based on provided file.
 * Definition of class Scene.
 */

#ifndef SCENE_HH_
#define SCENE_HH_

#include <cmath>
#include <vector>

#include "defs.hh"
#include "threading.hh"
#include "ray.hh"
//#include "object.hh"
#include "lights.hh"

//includes for compatibility with ex4a/ex4b
#include "sphere.hh"
#include "polygon.hh"

using std::vector;

/* class Scene.
 * The scene holds sets of all defined lights and objects.
 * It provides functions to trace rays between the objects.
 */
class Scene {
public:
  /* The constructor.
   * numReflections - number of rays to be sent to estimate each reflection/refraction.
   * reflectionsCutoff - the range of reflection rays (the cutoff angle in radians).
   */
  Scene(uint numReflections = 1, double reflectionsCutoff = 0.0)
    : _nReflections(numReflections), _tanRefCutoff(tan(reflectionsCutoff)),
      _ambientLight(Color3d(0.1, 0.1, 0.1)), _background(0.0)
  {
    if (_nReflections > 1) _refNormFactor = 1.0 / double(_nReflections);
  }

  /* Ray scene intersection for SHADOW RAYS.
   * Returns 0 - no intersection, 1 - in case of intersection
   * Set t for the ray parameter in intersection point.
   */
  int intersect_ray(Ray &ray, double &t);

  /* Ray scene intersection for COLOR RAYS.
   * Returns color seen through this ray.
   * In case of no intersection - returns the background color.
   * In case of intersection - returns shading at the point of intersection.
   */
  Color3d trace_ray(Ray &ray, double vis = 1.0);

  /* Calculates the shading model.
   * At point P, with surface normal N, viewed from direction V
   * Here ray tracing recursion occurs for transparent & reflective objects
   * at ray parameter t_min, with Object obj_min, and visibility factor vis
   * vis < 0.05 stops recursion.
   */
  Color3d shade(const Vector3d& V, const Point3d& P, const Vector3d& N,
                double t_min, Object* obj_min, double vis = 1.0);

  /*Add new object to the scene. */
  void add_object(Object * obj)
  {
    _objects.push_back(obj);
  }

  /*Add new light source to the scene. */
  void add_light(LocatedLight *light)
  {
    _lights.push_back(light);
  }
  
  /*Set background color for the scene.*/
  Color3d &backgroundColor()
  {
    return _background;
  }

  /* Set the ambient light source for the scene.*/
  AmbientLight &ambientLight()
  {
     return _ambientLight;
  }

  ~Scene();

private:
  typedef vector<Object *> ObjectsArray; //array of rendered objects
  typedef vector<LocatedLight *> LightsArray; //array of light sources
  
  uint _nReflections; //number of recursive rays per each reflection/refraction
  double _refNormFactor; //normalizing factor for multiple reflections average
  double _tanRefCutoff; //tangent of cutoff angle for multiple reflections

  ObjectsArray _objects; //scene objects
  LightsArray _lights; //scene lights

  AmbientLight _ambientLight; //the global ambient light
  Color3d _background; //scene background color

  /* Sends recursive ray(s) from point P in the direction of vector R.
   * Shoots as many reflection rays as defined in _nReflections parameter.
   * Returns averaged color estimation.
   * Used for reflection and refraction rays.
   */
  Color3d recursiveRay(const Point3d& P, const Vector3d& R,
                       const Color3d &component, double vis);

  /* Sends appropriate amount of recursive rays
   * in the direction of the given ray.
   * This function reuses and modifies the given ray object.
   */
  Color3d recursiveRay(Ray& ray, const Color3d &component, double vis);

  /* Computes refraction direction from the ray, send from V to point P,
   * where the normal is N.
   * obj - currently hit object,
   * vis - visibility factor for arrived ray.
   * leaving - true when ray leaving the object;
   *           false when the ray refracted inside the object.
   */
  Color3d shootRefraction(const Vector3d& V, const Point3d& P, const Vector3d& N,
                          Object *obj, double vis, bool leaving);
}; //class Scene

/*#####################Inline functions######################################*/

inline Scene::~Scene()
{
  //deallocating all lights and objects
  for (ObjectsArray::iterator it = _objects.begin(), end = _objects.end();
        it != end; ++it)
  {
    delete *it;
    *it = NULL;
  }
  for (LightsArray::iterator it = _lights.begin(), end = _lights.end();
        it != end; ++it)
  {
    delete *it;
    *it = NULL;
  }
}

//shadow rays
inline int Scene::intersect_ray(Ray &ray, double &t)
{
  Point3d P;
  double currT;
  t = INF;

  //iterate over all objects: test intersections
  for (ObjectsArray::const_iterator it = _objects.begin(), end = _objects.end();
       it != end; ++it)
  {
    if ((*it)->intersect(ray, currT, P, P) == 1 && currT < t)
    { //keep intersection with minimal t parameter: the closest one
      t = currT;
    }
  }
  return (t != INF);
}

//recursive rays
inline Color3d Scene::trace_ray(Ray &ray, double vis)
{
  assert(vis >= VIS_THRES); //called by recursiveRay() which checks vis threshold

  //variables for the current and the closest intersection
  Point3d p, minP;
  Vector3d n, minN;
  double t, minT = INF;
  Object *minObj = NULL;

  //iterate over all objects: test intersections
  for (ObjectsArray::const_iterator it = _objects.begin(), end = _objects.end();
    it != end; ++it)
  {
    if ((*it)->intersect(ray, t, p, n) == 1 && t < minT)
    { //found new closest intersection
      minT = t;
      minP = p;
      minN = n;
      minObj = (*it);
    }
  }

  if (minT == INF) return _background; //no intersections

  //estimate the shading in the intersection point (recursive calls)
  return shade(-ray.D(), minP, minN, minT, minObj, vis);
}

inline Color3d
Scene::recursiveRay(const Point3d& P, const Vector3d& R,
                       const Color3d &component, double vis)
{
  //new recursive ray
  Ray ray(P, R);
  ray._type = Ray::RAY_RECURSIVE;
  //send the ray and estimate the color
  return recursiveRay(ray, component, vis);
}


inline Color3d
Scene::recursiveRay(Ray& ray, const Color3d &component, double vis)
{
  //decrease visibility for next recursion level
  vis *= colorEnergy(component);
  if (vis < VIS_THRES) return COLOR_BLACK; //recursion stop condition

  if (_nReflections == 1)
  { //shoot single ray in exact R direction
    return component * trace_ray(ray, vis);
  }

  /*multiple rays handling*/

  //take some vector which is not parallel to ray direction
  Vector3d vec = ray.D() + Vector3d(0.5, 0.5, 0.5);
  //get any vector which is orthogonal to the ray
  Vector3d ortho = vec % ray.D();
  ortho.normalize();
  ortho *= _tanRefCutoff; //ortho length now equals to radius of cutoff
  ortho = vec_abs(ortho); //positive direction

  vec = ray.D(); //keep the original ray direction
  //now any randomly biased ray should be in range vec +- ortho

  //send first ray in the exact direction
  Color3d color = component * trace_ray(ray, vis);
  Vector3d dir(0.0);

  const uint tid = threadId(); //get id current thread

  //first random sample

  //pick random offset within the cutoff window
  dir = Vector3d(randomf(tid, -ortho[0], ortho[0]),
                 randomf(tid, -ortho[1], ortho[1]),
                 randomf(tid, -ortho[2], ortho[2]));
  dir += vec; //add the offset to the exact direction
  dir.normalize();
  ray.D() = dir;

  //send the ray
  Color3d estColor = component * trace_ray(ray, vis);

  if (approxEqual(estColor, color))
  { //random ray got nearly the same result as the direct ray
    //deducing what there's no more information to gain: fast return
    return color;
  }
  color += estColor;

  //send more _nReflections-2 rays
  for (uint i = 2; i < _nReflections; ++i)
  {
    //pick random offset within the cutoff window
    dir = Vector3d(randomf(tid, -ortho[0], ortho[0]),
                   randomf(tid, -ortho[1], ortho[1]),
                   randomf(tid, -ortho[2], ortho[2]));
    dir += vec; //add the offset to the exact direction
    dir.normalize();
    ray.D() = dir;

    //send the ray
    color += component * trace_ray(ray, vis);
  }
  //return normalized result: flat average
  return color * _refNormFactor;
}

inline Color3d
Scene::shootRefraction(const Vector3d& I, const Point3d& P, const Vector3d& N,
                       Object *obj, double vis, bool leaving)
{
  double index = obj->index();
  if (!leaving)
  { //entering from the air
    index = 1.0 / index;
  }

  double NdotI = I | N;
  double cosIn2 = 1.0 - sqr(index) * (1.0 - sqr(NdotI));
  if (cosIn2 < 0.0)
  { //internal reflection

    //ideal reflection from current viewing direction
    Vector3d R = N * (N | I) * 2.0 - I;
    Ray ray(P, R);

    Point3d dummy;
    double t;

    //find the exit intersection
    ray._type = Ray::RAY_INTERNAL;
    if (obj->intersect(ray, t, dummy, dummy))
    { //if the ray hits the object from inside.
      //translating it to begin from the exit point
      ray.O() = ray(t);
    }
    ray._type = Ray::RAY_RECURSIVE;
    //continue the recursion
    return recursiveRay(ray, obj->reflection(),  vis);
  }
  //finding the refraction vector
  Vector3d T = N * (index * NdotI - sqrt(cosIn2));
  T -= I * index;
  Ray ray(P, T);
  if (leaving)
  { //sending refraction ray from the exit point
    return recursiveRay(ray, obj->transparency(), vis);
  }
  else
  { //refracted ray enters the object
    Vector3d leavingI = -ray.D();
    Point3d leavingP;
    Vector3d leavingN;
    double t;
    //find the exit point
    if (obj->intersect(ray, t, leavingP, leavingN))
    {
      //compute refraction at the exit point
      return shootRefraction(leavingI, leavingP, -leavingN, obj, vis, true);
    }

    //else: no exit intersection, continuing refracted ray in the same direction
    return recursiveRay(ray, obj->transparency(), vis);
  }
}

#endif // SCENE_HH_

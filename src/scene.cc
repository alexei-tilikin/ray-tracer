/*
 * FILE: scene.cc
 * DESCRIPTION:
 * Implementation of class Scene.
 */
#include <cmath>

#include "scene.hh"

//ray recursion
Color3d Scene::shade(const Vector3d& V, const Point3d& P, const Vector3d& N,
               double t_min, Object* obj_min, double vis)
{
  //count in the ambient term
  Color3d color = ambientLight().color() * obj_min->ambient();

  Vector3d R, L; //ideal reflection vector, light direction vector
  double NdotL;

  //diffuse term in current point (texture mapping performed here)
  const Color3d diffuse_term = obj_min->diffuse(P);

  Color3d light_term(0.0); //color term for current light source

  //iterate over all light sources: send shadow rays
  for (LightsArray::const_iterator it = _lights.begin(), end = _lights.end();
        it != end; ++it)
  {
    light_term = (*it)->color(P, *this);
    //skip inactive lights
    if (approxEqual(light_term, COLOR_BLACK)) continue;

    //estimate the light direction vector
    L = ((*it)->position() - P).normalize();
    NdotL = N | L;
    //ideal reflection vector for the light
    R = (N * 2.0 * NdotL - L).normalize();
    //diffuse term
    color += light_term * diffuse_term * NdotL;
    //specular term
    color += light_term * obj_min->specular()
             * pow(double(V | R), obj_min->shining());

  }
  //Computing reflections
  R = N * (N | V) * 2.0 - V; //ideal reflection to view vector
  color += recursiveRay(P, R, obj_min->reflection(), vis);

  //Computing refractions
  color += shootRefraction(V, P, N, obj_min, vis, false);

  return color;
}

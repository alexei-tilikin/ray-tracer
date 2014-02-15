/*
 * FILE: object.hh
 * DESCRIPTION:
 * Based on provided file.
 * Definition and implementation of base Object class.
 */

#ifndef OBJECT_HH_
#define OBJECT_HH_

#include "defs.hh"
#include <EasyBMP.h>
#include "ray.hh"

/* class Object.
 * Abstract base class for all objects on scene.
 * Defines intersection test and texture mapping prototypes.
 * Holds all object-specific data.
 */
class Object {
 public:

  Object()
    : _diffuse(0, 0, 0), _specular(0, 0, 0), _ambient(0, 0, 0), _shining(25),
      _reflection(0, 0, 0), _transparency(0, 0, 0), _index(1.0),
      _diffuse_texture(NULL)
  {
  }

  /* Ray - object intersection.
   * Returns 0 if no intersection, 1 on intersection.
   * in:  ray - the ray to intersect with.
   * out: t - Distance from the ray origin to the intersection point.
   * out: P - Point in space of intersection.
   * out: N - Normal to Object at point of intersection.
   */
  virtual int
  intersect(Ray& ray, double& t, Point3d& P, Vector3d& N) = 0;

  /*Access functions to object colors.*/

  Color3d& diffuse()
  {
    return _diffuse;
  }
  Color3d& specular()
  {
    return _specular;
  }
  Color3d& ambient()
  {
    return _ambient;
  }
  Color3d& reflection()
  {
    return _reflection;
  }
  Color3d& transparency()
  {
    return _transparency;
  }

  /*Access functions to lighting factors.*/

  double& index()
  {
    return _index;
  }

  double& shining()
  {
    return _shining;
  }

  /* Returns diffuse term in point P of the object.
   * Take the texture mapping into account.
   */
  Color3d diffuse(const Point3d &P)
  {
    return _diffuse * texture_diffuse(P);
  }

  /* Retrieves texture color for certain point on the object.
   * Returns color from the texture mapping.
   * Returns pure white color if no texture available.
   */
  virtual Color3d texture_diffuse(const Point3d &P) = 0;

  /* Sets the given image to be a texture map for current object.
   * xTiling, yTiling are tiling parameters which determine how many times
   * the texture will repeat on each dimension.
   */
  void set_texture_map(BMP *image, double xTiling = 1.0,
                       double yTiling = 1.0);

  virtual ~Object()
  {
  }

protected:
  //color terms for lighting estimation
  Color3d _diffuse;
  Color3d _specular;
  Color3d _ambient;

  double _shining; //shininess coefficient
  Color3d _reflection; //reflected color term
  Color3d _transparency; //translucent color term

  double _index; //index of reflection

  BMP *_diffuse_texture; //the texture map
  Point2d _texBounds; //maximal coordinate on texture image
  //tiling factors for texture mapping
  double _texTilingU;
  double _texTilingV;

  /* Clips texture coordinates to [0, 1] range.
   * This function modifies the input!
   */
  TexCoord clipCoords(TexCoord &tex);

  /* Lookup the color value in texture map
   * for the given texture coordinates.
   * The coordinates origin set to be in left-bottom corner of image.
   */
  Color3d map_texture(TexCoord &tex);

}; //class Object

/*########################Inline functions###################################*/

inline void
Object::set_texture_map(BMP *image, double xTiling, double yTiling)
{
  if (image == NULL || image->TellWidth() < 1 || image->TellHeight() < 1)
  { //texture image unavailable
    ERR_POS;
    cerr << "Could not assign texture image to object.\n";
    return;
  }
  _diffuse_texture = image; //the texture
  //boundary on texture dimensions
  _texBounds = Point2d(_diffuse_texture->TellWidth() - 1,
                      _diffuse_texture->TellHeight() - 1);

  //setting the tiling factors (not allowed to be negative)
  _texTilingU = std::max(EPS, xTiling);
  _texTilingV = std::max(EPS, yTiling);
}

inline TexCoord Object::clipCoords(TexCoord &tex)
{
  if (tex[0] < 0.0) tex[0] = 0.0;
  else if (tex[0] > 1.0) tex[0] = 1.0;

  if (tex[1] < 0.0) tex[1] = 0.0;
  else if (tex[1] > 1.0) tex[1] = 1.0;

  return tex;
}

inline Color3d Object::map_texture(TexCoord &tex)
{
  //assuming the coordinates already clipped
  assert(tex[0] >= 0.0 && tex[0] <= 1.0);
  assert(tex[1] >= 0.0 && tex[1] <= 1.0);

  //apply tiling factors
  if (!approxEqual(_texTilingU, 1.0))
  {
    tex[0] *= _texTilingU;
    tex[0] -= floor(tex[0]);
  }
  if (!approxEqual(_texTilingV, 1.0))
  {
    tex[1] *= _texTilingV;
    tex[1] -= floor(tex[1]);
  }

  //get the pixel from the texture map
  //flipping Y axis direction
  RGBApixel *px = (*_diffuse_texture)(tex[0] * _texBounds[0],
                                  (1.0 - tex[1]) * _texBounds[1]);

  return Color3d(double(px->Red), double(px->Green), double(px->Blue)) / 255.0;
}

#endif // OBJECT_HH_

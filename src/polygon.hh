/*
 * FILE: polygon.hh
 * DESCRIPTION:
 * Based on provided file.
 * Defines the Polygon class.
 */

#ifndef POLYGON_HH_
#define POLYGON_HH_

#include <vector>

#include "defs.hh"
#include "threading.hh"
#include "object.hh"
#include "toolkits.hh"

using std::vector;

/* class Polygon.
 * Convex polygon object.
 * MUST be defined by at least 3 vertices.
 * The order of vertices affects the normal direction.
 */
class Polygon : public Object {
 public:

  /*Empty polygon: memory allocation only.*/
  Polygon()
  {
  }

  /* Define polygon by set of vertices and an arbitrary normal.
   * Texture mapping disabled.
   */
  Polygon(const vector<Point3d>& vertices, const Vector3d& normal)
  : _vertices(vertices), _normal(normal), _tex_coords(vertices.size())
  {
    computePoints();
  }

  /* Define polygon by set of vertices.
   * The normal computed automatically (counter-clockwise direction for front).
   * Texture mapping disabled.
   */
  Polygon(const vector<Point3d>& vertices)
    : _vertices(vertices), _tex_coords(vertices.size())
  {
    computePoints();
    computeNormal();
  }

  /* Define polygon by set of vertices, texture coordinates and arbitrary normal.
   * The number of texture coordinates must be equal to the number of vertices,
   * and naturally should be passed in the same order.
   */
  Polygon(const vector<Point3d>& vertices, const vector<TexCoord>& texcoords,
          const Vector3d& normal)
    : _vertices(vertices), _normal(normal), _tex_coords(texcoords)
  {
    computePoints();
  }

  /* Define polygon by set of vertices and texture coordinates.
   * The normal computed automatically (counter-clockwise direction for front).
   * The number of texture coordinates must be equal to the number of vertices,
   * and naturally should be passed in the same order.
   */
  Polygon(const vector<Point3d>& vertices, const vector<TexCoord>& texcoords)
    : _vertices(vertices), _tex_coords(texcoords)
  {
    computePoints();
    computeNormal();
  }

  /* Retrieves texture color for the given point P.*/
  virtual Color3d texture_diffuse(const Point3d &P);

  /* Convex polygon - ray intersection
   * t - the ray parameter at intersection
   * P - Point in space of intersection
   * N - Normal to Object at point of intersection
   * Returns 0 if no intersection.
   */
  virtual int intersect(Ray& ray, double& t, Point3d& P, Vector3d& N);

  virtual ~Polygon()
  {
  }

private:
  /* Information about the last hit triangle
   * Each thread has separate instance of LastHit.
   */
  struct LastHit
  {
    uint vertices[3];
    Point2d barycentric;
  };

  vector<Point3d> _vertices; //the polygon vertices
  Vector3d _normal; //the normal to the polygon plane
  vector<TexCoord> _tex_coords; //texture coordinates to all vertices
  Point3d _center; //the central point of polygon
  TexCoord _center_tex; //texture coordinate for the central point
  Point3d _minP; //minimal coordinates of the bounding box
  Point3d _maxP; //maximal coordinates of the bounding box

  //for each thread,
  //separate LastHit instance holds data about the last hit triangle
  LastHit _lastHit[NTHREADS];

  /* Set data in LastHit object.
   * Skipped if rayType == SHADOW_RAY.
   */
  void setLastHit(const Ray::Type &rayType,
                  uint tr0, uint tr1, uint tr2, const Point2d &barycentric);

  /* Computes polygon normal.
   * Uses _vertices data.
   */
  void computeNormal();

  /* Computes the center and the bounding coordinates of the polygon.
   * Uses _vertices data.
   */
  void computePoints();
}; //class Polygon

/*########################Inline functions###################################*/

inline void Polygon::computeNormal()
{
  _normal = ( _vertices[2] - _vertices[1]) % (_vertices[0] - _vertices[1]);
  _normal.normalize();
}

inline void Polygon::computePoints()
 {
   _minP = INF_POINT;
   _maxP = -_minP;
   _center = Point3d(0.0);
   _center_tex = TexCoord(0.0);

   double val;
   uint total = _vertices.size();

   //iterate over all vertices
   for (uint i = 0; i != total; ++i)
   {
     //accumulate all coordinates for center estimation
     _center += _vertices[i];
     _center_tex += _tex_coords[i];

     //update minimal & maximal coordinates when found
     for (uint j = 0; j < 3; ++j)
     {
       val = _vertices[i][j];
       if (val > _maxP[j]) _maxP[j] = val;
       if (val < _minP[j]) _minP[j] = val;
     }
   }
   _center /= (double) total;
   _center_tex /= (double) total;
 }

inline void
Polygon::setLastHit(const Ray::Type &rayType,
                    uint tr0, uint tr1, uint tr2, const Point2d &barycentric)
{
  if (rayType == Ray::RAY_SHADOW) return;

  uint tid = threadId();
  _lastHit[tid].vertices[0] = tr0;
  _lastHit[tid].vertices[1] = tr1;
  _lastHit[tid].vertices[2] = tr2;
  _lastHit[tid].barycentric = barycentric;
}

#endif // POLYGON_HH_

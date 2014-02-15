/*
 * FILE: polygon.cc
 * DESCRIPTION:
 * Implements the Polygon class.
 */
#include "polygon.hh"

int Polygon::intersect(Ray& ray, double& t, Point3d& P, Vector3d& N)
{
  //first find the intersection point with the plane
  if (!plane::intersect(_normal, _center, ray, t, P))
  { //the ray doesn't intersects with the plane
    return 0;
  }

  //the bounding box test
  if (P[0] + EPS < _minP[0] || P[0] - EPS > _maxP[0]
     || P[1] + EPS < _minP[1] || P[1] - EPS > _maxP[1]
     || P[2] + EPS < _minP[2] || P[2] - EPS > _maxP[2])
  { //out of polygon range
    return 0;
  }

  N = _normal; //setting the normal
  Point2d barycentric(0.0);

  /* Here the polygon triangulated on-the-fly and for each triangle
   * the estimated intersection point tested to lay inside the triangle.
   */
  assert(_vertices.size() > 2);

  /* Special cases:
   * A triangle doesn't need to be triangulated.
   * Quad (4-vertices) can be divided to 2 triangles, instead 4.
   */
  if (_vertices.size() <= 4)
  {
   if (triangle::fallsIn(P, (const Point3d&)_vertices[0],
                            (const Point3d&)_vertices[1],
                            (const Point3d&)_vertices[2], barycentric))
   { //the first triangle
     //remember the triangle indices and the barycentric coordinates of P
     setLastHit(ray._type, 0, 1, 2, barycentric);
     return 1;
   }

   if (_vertices.size() == 3) return 0;

   //else: _vertices.size() == 4
   else if (triangle::fallsIn(P, (const Point3d&)_vertices[2],
                                  (const Point3d&)_vertices[3],
                                  (const Point3d&)_vertices[0], barycentric))
   { //the second triangle of quad
     //remember the triangle indices and the barycentric coordinates of P
     setLastHit(ray._type, 2, 3, 0, barycentric);
     return 1;
   }
   return 0;
  }

  /* For all over polygons:
   * The center point used to divide n-sided polygon
   * into n triangles.
   */

  //iterate over all vertices
  //check if one of polygon triangles hold the intersection point
  for (uint curr = _vertices.size() - 1, next = 0, end = _vertices.size();
        next != end; curr = next, ++next)
  {
    if (triangle::fallsIn(P, (const Point3d&)_vertices[curr],
                              _center,
                             (const Point3d&)_vertices[next], barycentric))
    { //the point falls in the current triangle: it lays on the polygon

      //remember the triangle indices and the barycentric coordinates of P
      //the central vertex deduced from the polygon valence>4
      setLastHit(ray._type, curr, 0, next, barycentric);
      return 1;
    }
  }

  return 0; //the point is outside the polygon
}

//Implementing barycentrical mapping
Color3d Polygon::texture_diffuse(const Point3d &P)
{
  //no legal texture map
  if (_diffuse_texture == NULL
    || _tex_coords.size() != _vertices.size()) return COLOR_WHITE;


  //last hit data for current thread
  LastHit &lastHit = _lastHit[threadId()];

  //obtain texture coordinates for vertices from the stored indices
  const TexCoord& c0 = _tex_coords[lastHit.vertices[0]];
  const TexCoord& c1 = (_vertices.size() > 4)
                          ? _center_tex
                          : _tex_coords[lastHit.vertices[1]];
  const TexCoord& c2 = _tex_coords[lastHit.vertices[2]];

  //interpolate vertex texture coordinates, using barycentric coordinates of P
  TexCoord pos = triangle::interpolateBarycentric(lastHit.barycentric,
                                                  c0, c1, c2);
  clipCoords(pos); //clip texture coordinates
  return map_texture(pos); //retrieve the color from the texture
}

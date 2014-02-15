/*
 * FILE: MyMeshObject.hh
 * DESCRIPTION:
 * Based on provided file.
 * Definition of MyMeshObject class for Mesh-based objects.
 */

#ifndef MY_MESH_OBJECT_HH_
#define MY_MESH_OBJECT_HH_
#include <vector>

#include "defs.hh"
#include "threading.hh"
#include "MyMesh.hh"
#include "object.hh"
#include "BoundingSphere.hh"
#include "BoundingBox.hh"
#include "toolkits.hh"

/* class MyMeshObject.
 * Mesh of convex polygons.
 * The mesh defined by underlying OpenMesh object.
 * Several accelerators implemented to speed-up mesh rendering (see README).
 */
class MyMeshObject : public Object
{
public:

  /* The constructor.
   * mesh - the mesh object.
   * scaling - the uniform scaling factor for the mesh.
   */
  MyMeshObject(Mesh &mesh, double scaling = 1.0);

  /* Ray - Mesh intersection.
   * Implements virtual object::intersect(..).
   */
  virtual int intersect(Ray& ray, double& t, Point3d& P, Vector3d& N);

  /* Barycentric texture mapping for triangulated mesh.
   * Implements virtual object::texture_diffuse(..).
   */
  virtual Color3d texture_diffuse(const Point3d &P);

  virtual ~MyMeshObject();

private:
  /* Information about the last hit triangle.
   * Each rendering thread has separate instance of LastHit.
   */
  struct LastHit
  {
    Mesh::FaceHandle fHandle; //the triangle
    Point2d barycentric; //barycentric coordinates of last intersection
  };

  //array of mesh vertices
  typedef std::vector<Mesh::VertexHandle> VertexArray;

  //Bit access mask. Used on subspace division.
  static const uchar BIT_MASK1 = 01;
  static const uchar BIT_MASK2 = 02;
  static const uchar BIT_MASK3 = 04;

  /* Returns true iff two given subspaces
   * have a common plain, so they considered as neighbors (4-neighborhood).
   */
  static bool neighborSubspaces(uchar sp1, uchar sp2);

  Mesh _mesh; //the underlying mesh object
  Point3f _center; //the mass center of the mesh
  BoundingSphere _boundingSphere; //the bounding sphere
  BoundingBox _boundingBox; //the bounding box (for the whole mesh)
  BoundingBox _subSpaces[8]; //8 subspaces within the mesh bounding volume
  VertexArray _subSpacePartition[8]; //mesh vertices divided into 8 subspaces

  //for each thread,
  //separate LastHit instance holds data about the last intersection
  LastHit _lastHit[NTHREADS];

  /*Mesh properties*/
  OpenMesh::FPropHandleT<Point3f> propFMin; //minimal coordinates per face
  OpenMesh::FPropHandleT<Point3f> propFMax; //maximal coordinates per face

  //bounding box for group of neighbor faces (stored on the common vertex)
  OpenMesh::VPropHandleT<BoundingBox*> propVBox;

  /* Technical subroutine in space subdivision.
   * Returns number of subspace the point belongs to.
   */
  uchar detectSubspace(const Point3f &p);

  /* Returns true iff the point lays on one of the borders
   * between the defined subspaces.
   * Such border points should be treated specially.
   */
  bool onSubspaceBorder(const Point3f &p);

  /* Finds the subspace(s) where the given face belongs,
   * and adds the face into the subspace array(s).
   * The first vertex of the face added to _subSpacePartition array(s).
   */
  void addFaceToSubspace(const Mesh::FaceHandle &fHandle);

  /* Computes bounding objects for the whole mesh.
   * Also performs uniform scaling by modifying all vertex coordinates.
   */
  void computeBoundaries(double scaling);

  /* Computes relevant data for face properties.*/
  void computeFaceData();

  /* Computes relevant data for vertex properties.
   * Here the face neighborhood grouping performed.
   */
  void computeVertexData();

  /* Check certain face for intersection with the ray.
   * The face given by fHandle and assumed to be a triangle.
   * If intersection occurs: t, P, N, barycentric modified to hold
   * ray parameter, intersection point, the normal in the point,
   * and the barycentric coordinates of P on its face triangle.
   */
  bool intersectFace(const Mesh::FaceHandle &fHandle, const Ray& ray,
                     double& t, Point3d& P, Vector3d& N, Point2d &barycentric);

}; //class MyMeshObject

/*########################Inline functions###################################*/

inline bool MyMeshObject::neighborSubspaces(uchar sp1, uchar sp2)
{
  /* Two subspaces that share a plane may have only 1 different bit
   * in their numbers.
   * See function MyMeshObject::detectSubspace(..)
   * for subspace numbers convention.
   */
  uchar diff = sp1 ^ sp2;
  return (diff == BIT_MASK1 || diff == BIT_MASK2 || diff == BIT_MASK3);
}

inline MyMeshObject::MyMeshObject(Mesh &mesh, double scaling)
  : _mesh(mesh)
{
  _mesh.triangulate(); //working with triangle faces
  computeBoundaries(scaling); //bounding volumes
  computeFaceData(); //face boundaries
  computeVertexData(); //face neighborhood grouping, space subdivision
}

/* The subspace number convention:
 * range in [0, 7], 3 bits.
 * first bit: 0 for right, 1 for left;
 * second bit: 0 for top, 1 for bottom;
 * third bit: 0 for near, 1 for far;
 * The origin is the center-of-mass of the mesh.
 */
inline uchar MyMeshObject::detectSubspace(const Point3f &p)
{
  uchar subspace = 0;
  if (p[0] - _center[0] < 0.0) subspace |= BIT_MASK1;
  if (p[1] - _center[1] < 0.0) subspace |= BIT_MASK2;
  if (p[2] - _center[2] < 0.0) subspace |= BIT_MASK3;

  return subspace;
}

inline bool MyMeshObject::onSubspaceBorder(const Point3f &p)
{
  return (approxEqual(p[0], _center[0])
        || approxEqual(p[1], _center[1])
        || approxEqual(p[2], _center[2]) );
}

inline void
MyMeshObject::addFaceToSubspace(const Mesh::FaceHandle &fHandle)
{
  Mesh::ConstFaceVertexIter fvIt = _mesh.cfv_begin(fHandle);
  Mesh::VertexHandle vh0 = fvIt.handle(); //keep handle of first vertex
  Point3f v[3]; //vertex coordinates
  v[0] = _mesh.point(vh0);
  v[1] = _mesh.point((++fvIt).handle());
  v[2] = _mesh.point((++fvIt).handle());

  uchar subidx[3]; //subspaces of vertices
  subidx[0] = detectSubspace(v[0]); //find out the first vertex subspace

  /* Complex face is the one that suspected to intersect 3 subspaces.
   * Without trying to find out which subspaces it really intersects,
   * we simply add it to all 8 subspaces.
   * Because the division done around the mesh center,
   * there are small amount of complex faces expected.
   */
  bool complexFace = false;

  if (onSubspaceBorder(v[0]) || onSubspaceBorder(v[1]) || onSubspaceBorder(v[2]))
  { //if some of vertices lay on border, then the face is complex.
    complexFace = true;
  }
  else
  {
    //subspaces of all vertices
    subidx[1] = detectSubspace(v[1]);
    subidx[2] = detectSubspace(v[2]);

    //check each edge on the triangle (by pair of vertices)
    //looking for complex edges
    for (uint i = 0, j; i < 3; ++i)
    {
      j = addMod(i, uint, 1, 3);
      if (subidx[i] == subidx[j]) continue;

      if (!neighborSubspaces(subidx[i], subidx[j]))
      { /* Vertices lay on different non-neighbor subspaces.
         * That means the edge is pretty likely to intersect 3 subspace zones,
         * while the middle zone may be skipped on rendering.
         * To avoid this, we mark the face as complex.
         */
        complexFace = true;
        break;
      }
    } //end for
  }

  if (complexFace)
  { /* Complex faces added to all subspaces.
     * Only the first vertex of the face added.
     * It's enough because the intersection checked on groups of neighbor faces.
     */
    for (uchar i = 0; i < 8; ++i)
    {
      _subSpacePartition[i].push_back(vh0);
    }
    return;
  }
  //else: adding the face to the subspace of its first vertex (consistent convention)
  //again, only the first vertex of the face added to the partition
  _subSpacePartition[subidx[0]].push_back(vh0);
}

inline bool
MyMeshObject::intersectFace(const Mesh::FaceHandle &fHandle, const Ray& ray,
                            double& t, Point3d& P, Vector3d& N, Point2d &barycentric)
{
  N = _mesh.normal(fHandle);
  Mesh::ConstFaceVertexIter fvIt = _mesh.cfv_iter(fHandle);

  //using any point on the face to define the plane
  if (!plane::intersect(N, Point3d(_mesh.point(fvIt)), ray, t, P))
  { //the ray doesn't intersects with the plane
    return false;
  }

  //now we got P - the intersection point with the face' plane

  const Point3d minP(_mesh.property(propFMin, fHandle));
  const Point3d maxP(_mesh.property(propFMax, fHandle));

  //the bounding box test for current face
  if (P[0] + EPS < minP[0] || P[0] - EPS > maxP[0]
      || P[1] + EPS < minP[1] || P[1] - EPS > maxP[1]
      || P[2] + EPS < minP[2] || P[2] - EPS > maxP[2])
  {
    return false;
  }

  //the accurate test for P to fall inside the face (which is a triangle)
  Point3d v0, v1, v2; //all three vertices
  v0 = _mesh.point(fvIt);
  v1 = _mesh.point(++fvIt);
  v2 = _mesh.point(++fvIt);

  return triangle::fallsIn(P, v0, v1, v2, barycentric);
}

#endif //MY_MESH_OBJECT_HH_

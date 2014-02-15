/*
 * FILE: MyMeshObject.cc
 * DESCRIPTION:
 * Implementation of MyMeshObject class.
 */
#include <algorithm>

#include "MyMeshObject.hh"

using namespace std;

MyMeshObject::~MyMeshObject()
{
  //deallocating BoundingBox objects on group of faces
  BoundingBox *box = NULL;
  for (Mesh::VertexIter vIt = _mesh.vertices_begin(),
        vEnd = _mesh.vertices_end(); vIt != vEnd; ++vIt)
  {
    box = _mesh.property(propVBox, vIt.handle());
    delete box;
  }
  _mesh.remove_property(propVBox);
}

void MyMeshObject::computeBoundaries(double scaling)
{
  //calculating the minimal and the maximal coordinates, and the center-of-mass
  Point3f minP = Point3f(INF_POINT);
  Point3f maxP = - minP;
  _center = Point3f(0.0);
  Point3f p;
  Mesh::VertexHandle vHandle;

  //iterate over all mesh vertices
  for (Mesh::VertexIter vIt = _mesh.vertices_begin(),
                              vEnd = _mesh.vertices_end();
      vIt != vEnd; ++vIt)
  {
    vHandle = vIt.handle();
    //applying scaling factor right on the mesh
    p = _mesh.point(vIt) * scaling;
    _mesh.set_point(vHandle, p);
    _center += p; //accumulate all vertices for center-of-mass

    for (uint i = 0; i < 3; ++i)
    {
      if (p[i] > maxP[i]) maxP[i] = p[i];
      if (p[i] < minP[i]) minP[i] = p[i];
    }
  }
  _center /= double(_mesh.n_vertices());

  //bounding sphere from the center point with radius as distance to the far corner
  _boundingSphere = BoundingSphere(Point3d(_center),
                                   std::max((maxP - _center).length(),
                                            (minP - _center).length()));
  //bounding box for the whole mesh
  _boundingBox = BoundingBox(minP, maxP);

  //computing bounding boxes for sub-spaces

  //right-top-near
  _subSpaces[0] = BoundingBox(_center, maxP);
  //left-top-near
  _subSpaces[1] = BoundingBox(Point3f(minP[0], _center[1], _center[2]),
                              Point3f(_center[0], maxP[1], maxP[2]));
  //right-bottom-near
  _subSpaces[2] = BoundingBox(Point3f(_center[0], minP[1], _center[2]),
                              Point3f(maxP[0], _center[1], maxP[2]));
  //left-bottom-near
  _subSpaces[3] = BoundingBox(Point3f(minP[0], minP[1], _center[2]),
                              Point3f(_center[0], _center[1], maxP[2]));
  //right-top-far
  _subSpaces[4] = BoundingBox(Point3f(_center[0], _center[1], minP[2]),
                              Point3f(maxP[0], maxP[1], _center[2]));
  //left-top-far
  _subSpaces[5] = BoundingBox(Point3f(minP[0], _center[1], minP[2]),
                              Point3f(_center[0], maxP[1], _center[2]));
  //right-bottom-far
  _subSpaces[6] = BoundingBox(Point3f(_center[0], minP[1], minP[2]),
                              Point3f(maxP[0], _center[1], _center[2]));
  //left-bottom-far
  _subSpaces[7] = BoundingBox(minP, _center);

}

/* Weak ordering on mesh vertices.
 * Using on _subSpacePartition sort.
 */
static bool compareVHandle(const Mesh::VertexHandle &vh1,
                    const Mesh::VertexHandle &vh2)
{
  return (vh1.idx() < vh2.idx());
}

/* Returns true iff two handles point to the same vertex.
 * Used on duplicates removing from _subSpacePartition with unique(..).
 */
static bool equalVHandles(const Mesh::VertexHandle &vh1,
                          const Mesh::VertexHandle &vh2)
{
  return (vh1.idx() == vh2.idx());
}

void MyMeshObject::computeFaceData()
{
  //add face properties on the mesh
  _mesh.add_property(propFMin);
  _mesh.add_property(propFMax);

  //increase performance by reducing number of vector reallocations
  const uint predictVerticesPerSubspace = _mesh.n_vertices() / 8.0;
  for (uchar i = 0; i < 8; ++i)
  {
    _subSpacePartition[i].reserve(predictVerticesPerSubspace);
  }

  Point3f p;
  Point3f minP, maxP;
  Mesh::FaceHandle fHandle;

  //iterate over faces
  for (Mesh::FaceIter fIt = _mesh.faces_begin(), fEnd = _mesh.faces_end();
        fIt != fEnd; ++fIt)
  {
    fHandle = fIt.handle();

    //add current face to its subspace partition(s)
    addFaceToSubspace(fHandle);

    minP = Point3f(INF_POINT);
    maxP = -minP;

    //iterate over all vertices on current face:
    //get minimal and maximal coordinates among all vertices of the face
    for (Mesh::ConstFaceVertexIter fvIter = _mesh.cfv_iter(fHandle);
            fvIter; ++fvIter)
    {
      p = _mesh.point(fvIter);
      for (uint i = 0; i < 3; ++i)
      {
        if (p[i] > maxP[i]) maxP[i] = p[i];
        if (p[i] < minP[i]) minP[i] = p[i];
      }
    }
    _mesh.property(propFMin, fHandle) = minP;
    _mesh.property(propFMax, fHandle) = maxP;
  }

  //iterate over all subspace partitions
  for (uchar i = 0; i < 8; ++i)
  {
    //sort all vertices on partition
    sort(_subSpacePartition[i].begin(), _subSpacePartition[i].end(), compareVHandle);
    //remove duplicated vertices
    VertexArray::iterator last = unique(_subSpacePartition[i].begin(),
                                        _subSpacePartition[i].end(),
                                        equalVHandles);
    //resize the partition vector to match size without the duplicates
    _subSpacePartition[i].resize(distance(_subSpacePartition[i].begin(), last));
  }

  //set face normals
  _mesh.request_face_normals();
  _mesh.update_normals();
}

void MyMeshObject::computeVertexData()
{
  //enable vertex properties
  _mesh.add_property(propVBox);

  Point3f minP, maxP, currMin, currMax;
  Mesh::VertexHandle vHandle;
  Mesh::FaceHandle fHandle;

  //iterate over vertices
  for (Mesh::VertexIter vIt = _mesh.vertices_begin(), vEnd = _mesh.vertices_end();
      vIt != vEnd; ++vIt)
  {
    vHandle = vIt.handle();
    minP = Point3f(INF_POINT);
    maxP = -minP;

    //iterate over faces that share the current vertex:
    //get minimal and maximal corners among all neighbor faces
    for (Mesh::ConstVertexFaceIter vfIt = _mesh.cvf_iter(vHandle); vfIt; ++vfIt)
    {
      fHandle = vfIt.handle();
      currMin = _mesh.property(propFMin, fHandle);
      currMax = _mesh.property(propFMax, fHandle);

      for (uint i = 0; i < 3; ++i)
      {
        if (currMin[i] < minP[i]) minP[i] = currMin[i];
        if (currMax[i] > maxP[i]) maxP[i] = currMax[i];
      }
    }
    //store the bounding box for this group of neighbor faces
    _mesh.property(propVBox, vHandle) = new BoundingBox(minP, maxP);
  }
}


int MyMeshObject::intersect(Ray& ray, double& t, Point3d& P, Vector3d& N)
{
  //bounding volumes test: skipped for internal rays
  if (ray._type != Ray::RAY_INTERNAL)
  {
    double t1, t2;
    //easy sphere-ray test
    if (_boundingSphere.intersect(ray, t1, t2) == 0) return 0;
    //more exact box-ray test
    if (!_boundingBox.intersect(ray)) return 0;
  }

  //set of variables to keep last intersection candidate
  double currT;
  Vector3d currN(0.0);
  Point3d currP(0.0);
  Point2d barycentric(0.0);

  //variables in use in loops
  BoundingBox *box = NULL;
  Mesh::VertexHandle vHandle;
  Mesh::ConstVertexIter vIt, vEnd;
  Mesh::ConstVertexFaceIter vfIt;

  //last hit data for current thread
  LastHit &lastHit = _lastHit[threadId()];

  t = INF; //preparing to accept any positive t

  /*for each subspace, check whether the ray intersects it,
   * and only then continue with all the vertices on that subspace.
   */
  for (uchar i = 0; i < 8; ++i)
  {
    /* Each potentially hit face should belong to some L2 group
     * with vertex in some intersected subspace.
     * If current subspace not intersected by the ray, it may be skipped.
     */
    if (!_subSpaces[i].intersect(ray)) continue;

    /* Iterating over vertices of the intersected subspace.
     * Each vertex holds bounding box for group of faces.
     */
    for (VertexArray::const_iterator it = _subSpacePartition[i].begin(),
                                     end = _subSpacePartition[i].end();
        it != end; ++it)
    {
      vHandle = *it;
      box = _mesh.property(propVBox, vHandle);
      assert(box != NULL);
      if (!box->intersect(ray))
      { //the ray doesn't intersect with the group of faces
        continue; //skipping to the next group
      }

      //iterate over faces=triangles in the group:
      //check each triangle for intersection
      for (vfIt = _mesh.cvf_iter(vHandle); vfIt; ++vfIt)
      {
        if (!intersectFace(vfIt.handle(), ray, currT, currP, currN, barycentric)
            || currT >= t)
        { //no intersection, or not the closest intersection
          continue;
        }

        //update intersection data (maybe this one will be the closest)
        t = currT;
        N = currN;
        P = currP;

        if (ray._type != Ray::RAY_SHADOW)
        {
          //remember last face there the intersection occurred
          lastHit.fHandle = vfIt.handle();
          //remember barycentric coordinates of last hit point
          lastHit.barycentric = barycentric;
        }

      } //end for vfIt
    } //end for _subSpaceDivision[i]
  } //end for subspaces

  return (t != INF);
}

Color3d MyMeshObject::texture_diffuse(const Point3d &P)
{
  //fast return when texture mapping unavailable
  if (_diffuse_texture == NULL || !_mesh.has_vertex_texcoords2D())
  {
    return COLOR_WHITE;
  }

  //last hit data for current thread
  const LastHit &lastHit = _lastHit[threadId()];
  assert(lastHit.fHandle.is_valid()); //assuming there was some intersection

  Mesh::ConstFaceVertexIter fvIt = _mesh.cfv_iter(lastHit.fHandle);

  //set vertex handles for triangle vertices
  Mesh::VertexHandle v0, v1, v2;
  v0 = fvIt.handle();
  v1 = (++fvIt).handle();
  v2 = (++fvIt).handle();

  //interpolate texture coordinates of vertices
  //using the barycentic coordinates of current point
  TexCoord pos = triangle::interpolateBarycentric(lastHit.barycentric,
                                                TexCoord(_mesh.texcoord2D(v0)),
                                                TexCoord(_mesh.texcoord2D(v1)),
                                                TexCoord(_mesh.texcoord2D(v2)) );
  clipCoords(pos); //clip the texture coordinates
  return map_texture(pos); //retrieve the color from the texture
}

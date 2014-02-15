/*
 * FILE: MyMesh.hh
 * DESCRIPTION:
 * Definitions of Mesh and MyMesh types.
 * Keeping for compatibility.
 */

#ifndef OPEN_MESH_DEFS_HH_
#define OPEN_MESH_DEFS_HH_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;
typedef MyMesh Mesh;

#endif // OPEN_MESH_DEFS_HH_

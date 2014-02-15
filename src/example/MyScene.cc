/*
 * FILE: MyScene.cc
 * AUTHORS: schnider, srfd45
 * COURSE: Computer Graphics
 * PROJECT: EX4
 * DESCRIPTION:
 * Our custom scene, compiles into ex4.MyScene executable.
 * Using spherical lights, multiple reflections.
 * See README for explanation about the executable arguments.
 */
#include <cstdlib>
#include <cmath>
#include <iostream>

#include "defs.hh"
#include "scene.hh"
#include "camera.hh"
#include "sphere.hh"
#include "polygon.hh"
#include "MyMeshObject.hh"
#include "Cylinder.hh"
#include "Arguments.hh"

using namespace std;

/* define some colors */
static const Color3d white(1.0, 1.0, 1.0);
static const Color3d red(1, 0, 0.0);
static const Color3d green(0, 1.0, 0.0);
static const Color3d blue(0, 0, 1.0);
static const Color3d yellow(1.0, 1.0, 0.0);
static const Color3d colorSand(1.0, 0.659, 0.345); //255, 168, 88

//texture image objects
static BMP *floor_tex = NULL;
static BMP *pyramid_tex = NULL;
static BMP *sun_tex = NULL;
static BMP *col_tex = NULL;
static BMP *earth_tex = NULL;

//path to resources directory. Value overwritten in main()
static string resourcePath = "../../";

/* The scene lights.*/
static void defineLights(Scene *scene)
{
  Point3d pos;
  Color3d color;

  scene->ambientLight() = AmbientLight(white * 0.8);
  scene->backgroundColor() = blue * 0.1;

  //main light above the sun
  pos = Point3d(0, 35, 0);
  LocatedLight * above = new SphereLight(pos, 10.0, yellow * 0.7 + white * 0.1);
  scene->add_light(above);

  //left lights colored red
  color = red * 0.4 + white * 0.6;
  pos = Point3d(-2, 1.5, 4);
  LocatedLight * front_left = new SphereLight(pos, 1.5, color * 0.4);
  scene->add_light(front_left);

  pos = Point3d(-2, 2, -2.1);
  LocatedLight * back_left = new SphereLight(pos, 1.5, color);
  scene->add_light(back_left);

  //right lights colored green
  color = green * 0.4 + white * 0.6;
  pos = Point3d(2, 1.5, 4);
  LocatedLight * front_right = new SphereLight(pos, 1.5, color * 0.4);
  scene->add_light(front_right);

  pos = Point3d(2, 2, -2.1);
  LocatedLight * back_right = new SphereLight(pos, 1.5, color);
  scene->add_light(back_right);
}

/*The objects on scene*/
static void defineGeometry(Scene *scene)
{
  Point3d pos;
  Color3d color;
  double radius, height;
  double offset = 0.0;

  //loading texture images. Verifying successful load.
  floor_tex = loadBMP(resourcePath + "/textures/floor.bmp");
  pyramid_tex = loadBMP(resourcePath + "/textures/pyramid.bmp");
  sun_tex = loadBMP(resourcePath + "/textures/sun.bmp");
  col_tex = loadBMP(resourcePath + "/textures/column.bmp");
  earth_tex = loadBMP(resourcePath + "/textures/Earth.bmp");

  //the pyramid mesh
  MyMesh mesh;
  mesh.request_vertex_texcoords2D();
  if (!OpenMesh::IO::read_mesh(mesh, resourcePath + "/models/pyramid.obj"))
  {
    cerr << "Error: cannot read mesh from file\n";
    return;
  }

  MyMeshObject *pyramid = new MyMeshObject(mesh, 2.0);
  pyramid->ambient() = colorSand * 0.1;
  pyramid->diffuse() = white * 0.7; //red * 0.2 + white * 0.1; //white
  pyramid->reflection() = white * 0.1;
  pyramid->specular() = white * 0.4;
  pyramid->shining() = 2.0;
  //pyramid->index() = 0.5;
  //pyramid->transparency() = white * 0.6;
  pyramid->set_texture_map(pyramid_tex, 2.0, 2.5);
  scene->add_object(pyramid);

  //the Earth sphere
  radius = 4;
  pos = Point3d(-20, 20, -30);
  Sphere * earth = new Sphere(pos, radius);
  earth->ambient() = (blue * 0.6 + white * 0.4) * 0.3;
  earth->diffuse() = white * 0.8;
  earth->reflection() = white * 0.2;
  earth->specular() = white * 0.3;
  earth->shining() = 200;
  earth->set_texture_map(earth_tex);
  scene->add_object(earth);

  //the sun sphere
  radius = 0.8;
  pos = Point3d(0, 3.0+radius, 0);
  Sphere * sun = new Sphere(pos, radius);
  sun->ambient() = (red * 0.3 + yellow * 0.2 + white * 0.5) * 0.1;
  sun->diffuse() = white * 0.9;
  sun->reflection() = white * 0.8;
  sun->set_texture_map(sun_tex, 1.0, 2.0);
  scene->add_object(sun);

  //the columns
  radius = 0.15;
  height = 3.0;

  //left column
  pos = Point3d(-2.5, offset, 1.9);
  Cylinder *col_left = new Cylinder(pos, radius, height - offset);
  col_left->diffuse() =  white;
  col_left->reflection() = white * 0.2;
  col_left->specular() = white * 0.5;
  col_left->shining() = 70.0;
  col_left->set_texture_map(col_tex, 1.0, 2.0);
  scene->add_object(col_left);

  //right column
  pos = Point3d(2.5, offset, 1.9);
  Cylinder *col_right = new Cylinder(pos, radius, height - offset);
  col_right->diffuse() = white;
  col_right->reflection() = white * 0.2;
  col_right->specular() = white * 0.5;
  col_right->shining() = 70.0;
  col_right->set_texture_map(col_tex, 1.0, 2.0);
  scene->add_object(col_right);


  radius = 0.2;
  height = 0.1;

  //left column cap
  pos = Point3d(-2.5, offset + 3.0, 1.9);
  Cylinder *cap_left = new Cylinder(pos, radius, height);
  cap_left->diffuse() = colorSand;
  cap_left->reflection() = white * 0.2;
  //cap_left->index() = 1.5;
  cap_left->specular() = white * 0.5;
  cap_left->shining() = 70.0;
  scene->add_object(cap_left);

  //right column cap
  pos = Point3d(2.5, offset + 3.0, 1.9);
  Cylinder *cap_right = new Cylinder(pos, radius, height);
  cap_right->diffuse() = colorSand;
  cap_right->reflection() = white * 0.2;
  cap_right->specular() = white * 0.5;
  cap_right->shining() = 70.0;
  scene->add_object(cap_right);

  //the floor plane
  color = blue * 0.3 + white * 0.7;

  vector<Point3d> plane(4);
  vector<Point2d> plane_uv(4);
  double x = 30;
  double y = offset;
  plane[0] = Point3d(-x, y, -x);
  plane[1] = Point3d(-x, y, x);
  plane[2] = Point3d(x, y, x);
  plane[3] = Point3d(x, y, -x);
  plane_uv[0] = Point2d(0, 0);
  plane_uv[1] = Point2d(0, 1);
  plane_uv[2] = Point2d(1, 1);
  plane_uv[3] = Point2d(1, 0);
  Polygon *poly = new Polygon(plane, plane_uv);
  poly->ambient() = blue * 0.2;
  poly->diffuse() = white * 0.5;
  poly->reflection() = white * 0.5;
  poly->set_texture_map(floor_tex, 9.0, 9.0);
  scene->add_object(poly);

  //anchor coordinates for mirrors
  double far = -9.0, near = 2.0,
         left = -4.0, right = 4.0,
         floor = 0.0, ceil = 4.0;

  //left mirror
  plane = vector<Point3d>(4);

  plane[0] = Point3d(0, floor, far);
  plane[1] = Point3d(0, ceil, far);
  plane[2] = Point3d(left, ceil, near);
  plane[3] = Point3d(left, floor, near);

  Polygon *mirror_left = new Polygon(plane);
  mirror_left->reflection() = white * 0.8;
  mirror_left->diffuse() = white * 0.1;
  scene->add_object(mirror_left);

  //right mirror
  plane = vector<Point3d>(4);

  plane[0] = Point3d(0, ceil, far);
  plane[1] = Point3d(0, floor, far);
  plane[2] = Point3d(right, floor, near);
  plane[3] = Point3d(right, ceil, near);

  Polygon *mirror_right = new Polygon(plane);
  mirror_right->reflection() = white * 0.8;
  mirror_right->diffuse() = white * 0.1;
  scene->add_object(mirror_right);
}

int main(int argc, const char* argv[])
{
  Arguments args(argc, argv);
  args.parse();
  args.print();
  resourcePath = args.resourcePath;
    
  //setting the scene
  Scene *scene = new Scene(args.numReflections, args.reflectionsCutoff);
  defineGeometry(scene);
  defineLights(scene);

  //setting the camera
  Point3d pos(1, 2, 7);
  Point3d coi(0, 1.5, 0);
  Vector3d up(0, 1, 0);
  double fov_h = deg2rad(40.0);
  Camera camera(pos, coi, up, fov_h, args.samplesPerPixel);

  //rendering
  camera.render(args.height, args.width, "MyScene.bmp", *scene);
  delete scene;

  //deallocating textures
  delete floor_tex;
  delete pyramid_tex;
  delete sun_tex;
  delete col_tex;
  delete earth_tex;

  return RC_OK;
}

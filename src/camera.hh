/*
 * FILE: camera.hh
 * DESCRIPTION:
 * Based on provided file.
 * Definition of class Camera.
 */
#ifndef CAMERA_HH_
#define CAMERA_HH_

#include <EasyBMP.h>
#include "defs.hh"
#include "threading.hh"
#include "scene.hh"
#include "rotation.hh"

/* class Camera.
 * Provides multithreaded rendering function,
 * which creates an output image file for the given Scene object.
 */
class Camera
{
public:
  /* Create new camera instance.
   * pos - position of the camera.
   * coi - the center point on the resulting image: the point where the camera faces.
   * up - vector of camera up direction (defines the tilt angle).
   * fovH - the angle for horizontal field of view (in radians).
   * supersampling - number of initial rays to send per each pixel.
   */
  Camera(Point3d &pos, Point3d &coi, Vector3d &up,
         double fovH, uint supersampling = 1)
  : _position(pos), _coi(coi), _up(up.normalized()), _fov_h(fovH),
    _samplesPerPixel(supersampling), _invSamplesPerPixel(1.0 / double(supersampling)),
    _threadData(NULL)
  {
  }

  /* Starting point for the rendering task.
   * This function may adaptively launch multiple rendering threads.
   * height, width - the dimensions of the output image.
   * fileName - the name of the output image file.
   * scene - the scene object to render.
   */
  void render(int height, int width, const char* fileName, Scene &scene);

private:

  /* Holder for all the data that multiple rendering threads
   * access concurrently.
   */
  struct ThreadData
  {
    ulong totalPixels; //total number of pixels on current image
    ulong nextPixel; //next untreated pixel
    double progress; //progress indicator
    Lock counterLock; //lock on nextPixel and progress values
    double cameraDistance; //the distance between the camera and the center of view
    double pixelW; //width of single pixel on the image plane
    double pixelH; //height of single pixel on the image plane
    Rotation rotationUp; //rotation for tilt alignment
    Rotation rotationView; //rotation for center-of-view alignment.
    Point2d dirAlignment; //additive factors to move the image center to the camera center.
    Vector3d samplingOffset; //offset vector for super-sampling calculations
    Scene *scene; //currently rendered scene
    BMP *outImage; //the output image
    Lock imageLock; //the lock for write access on the output image
    pthread_t threads[NTHREADS - 1]; //identifiers of all running threads

    ThreadData()
      : nextPixel(0), progress(0.0), counterLock(), outImage(NULL), imageLock()
    {
    }

    ~ThreadData()
    {
      delete outImage;
    }
  };

  /* Wrapper for rendering thread entry point.
   * Gets instance of Camera as argument and starts new thread.
   */
  static void* thread_start(void* instance);

  /* Camera internal fields*/
  Point3d _position;    //Position of the camera in the 3D space
  Point3d _coi;         //Center of interest for the camera
  Vector3d _up;         //Vector pointing up
  double _fov_h;        //Horizontal field of view for the camera
  uint _samplesPerPixel; //number of samples to take for each pixel
  double _invSamplesPerPixel; //(1.0 / _samplesPerPixel) for faster normalization
  ThreadData *_threadData; //the shared data for rendering threads

  /*Rendering thread routine.*/
  void renderThread();

  /* Rendering subroutine: renders a single pixel.
   * Sends multiple rays when supersampling requested.
   * Returns the color estimation for the pixel.
   * x, y - the pixel coordinates on the resulting image;
   * ray - reused ray object with initialized origin ray.O();
   * tid - Id of the calling thread (as returned by threadId() );
   */
  Color3d renderPixel(uint x, uint y, Ray &ray, uint tid);
}; //class Camera

/*#####################Inline functions######################################*/

inline Color3d
Camera::renderPixel(uint x, uint y, Ray &ray, uint tid)
{
  ThreadData &data = *_threadData;
  Color3d color(0.0);

  //computing the direction vector for the center of the pixel
  //Y axis flipped by taking negative y coordinate
  Vector3d dir(data.pixelW * (double(x) + data.dirAlignment[0]),
               data.pixelH * (data.dirAlignment[1] - double(y)),
               data.cameraDistance);

  //apply needed rotations to align the direction
  data.rotationUp.rotate(dir);
  data.rotationView.rotate(dir);

  //trace first ray in the pixel center (the direct ray)
  ray.D() = dir.normalized();
  color += data.scene->trace_ray(ray, 1.0);

  //if no supersampling - return the color
  if (_samplesPerPixel == 1) return color;

  //supersampling: first send one random ray and compare result to the direct ray
  Vector3d dirOffset = data.samplingOffset;
  dirOffset[0] *= randomf(tid, -1.0, 1.0); //randomize X offset
  dirOffset[1] *= randomf(tid, -1.0, 1.0); //randomize Y offset
  dirOffset += dir; //translate current direction vector by the offset
  ray.D() = dirOffset.normalize();
  Color3d estColor = data.scene->trace_ray(ray, 1.0); //render the ray

  if (approxEqual(estColor, color))
  { //random ray got nearly the same result as the direct ray
    //deducing what there's no more information to gain: fast return
    return color;
  }
  color += estColor;

  //send the rest of random rays
  for (uint s = 2; s < _samplesPerPixel; ++s)
  {
    dirOffset = data.samplingOffset;
    dirOffset[0] *= randomf(tid, -1.0, 1.0); //randomize X offset
    dirOffset[1] *= randomf(tid, -1.0, 1.0); //randomize Y offset
    dirOffset += dir; //translate current direction vector by the offset
    ray.D() = dirOffset.normalize();
    color += data.scene->trace_ray(ray, 1.0); //render the ray
  }

  color *= _invSamplesPerPixel; //normalize accumulated color: flat average
  return color;
}

#endif // CAMERA_HH_

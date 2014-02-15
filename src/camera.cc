/*
 * FILE: camera.cc
 * DESCRIPTION:
 * Implementation of class Camera.
 */
#include "camera.hh"

using namespace std;

void
Camera::render(int height, int width, const char* fileName, Scene &scene)
{

  static const Vector3d canonicalUp = Vector3d(0, 1, 0);
  //facing vector from the camera to the center-of-view
  static const Vector3d canonicalFacing = Vector3d(0, 0, 1);

  if (strlen(fileName) == 0)
  {
    cerr << "Camera::render: Empty output file name.\n";
    exit(RC_INVALID_ARGUMENTS);
  }

  const double w = double(width);
  const double h = double(height);
  const double aspect = h / w;
  //the viewing vector: from the camera to the center-of-view
  Point3d view = (_coi - _position);
  //total distance between camera and center-of-view
  double dist = view.norm();
  //determining the sign of direction vectors
  if (view[2] < 0.0) dist = -dist;

  //determining the size of projection grid
  //X axis flipped when looking in positive Z direction (from negative side)
  const double proj_width = tan(_fov_h) * 2.0 * -dist;
  //Y axis not affected by the Z direction (it always flipped)
  const double proj_height = tan(_fov_h * aspect) * 2.0 * fabs(dist);

  /* ray alignment on projection grid: 1 + half-pixel + half window
   * coi becomes the center of the projection grid.
   * Y-axis flipped here to match the image coordinates.
   */
  const Point2d alignment(1.5 - double(width/2),  h + 1.5 - double(height/2));

  //tilt rotation of projection plane according _up vector
  Rotation rotationUp(_up, canonicalUp);

  //set the canonical facing
  Vector3d facing = canonicalFacing;
  //determine the direction
  if (view[2] < 0.0) facing[2] = -facing[2];

  //camera rotation to face to the center-of-view
  Rotation rotationView(facing, view);

  //size of single pixel on the projection grid
  const double pixelW = proj_width / w;
  const double pixelH = proj_height / h;

  Vector3d samplingOffset(0.0);
  if (_samplesPerPixel > 1)
  {
    /* Offset vector to perform translations from the pixel center.
     * The offset represented in projection grid coordinates
     * with origin in the middle of current pixel.
     * Any translations by this vector with factors [-1, 1]
     * keep the point inside the pixel window.
     */
    samplingOffset = Vector3d(0.5 * pixelW,
                              0.5 * pixelH,
                              0.0);

    //align the offset vector with the tilt rotation
    rotationUp.rotate(samplingOffset);
  }

  //Initialize thread data
  _threadData = new ThreadData();
  //copy all computations
  _threadData->totalPixels = ulong(width) * ulong(height);
  _threadData->cameraDistance = dist;
  _threadData->pixelW = pixelW;
  _threadData->pixelH = pixelH;
  _threadData->rotationUp = rotationUp;
  _threadData->rotationView = rotationView;
  _threadData->dirAlignment = alignment;
  _threadData->samplingOffset = samplingOffset;
  _threadData->scene = &scene;
  _threadData->outImage = new BMP();
  _threadData->outImage->SetSize(width, height);
  _threadData->outImage->SetBitDepth(24);

  //start rendering
  cout << "Rendering..." << endl;

  if (_threadData->totalPixels <= THREAD_SERV_CHUNK)
  { //small image: serve all in one thread
    renderThread();
    if (!_threadData->outImage->WriteToFile(fileName))
    {
      cerr << "Error while writing to the output image file.\n";
      exit(RC_INPUT_ERROR);
    }
    delete _threadData;
    cout << "\rAll done." << endl;
    return;
  }

  //creating multiple rendering threads
  for (uint i = 0; i < NTHREADS - 1; ++i)
  {
    //create new thread
    if (pthread_create(_threadData->threads + i, NULL, thread_start, (void*)this))
    {
      cerr << "Error while creating thread!" << endl;
      delete _threadData;
      exit(RC_UNKNOWN_ERROR);
    }
  }

  //main thread also does the job
  renderThread();

  /* Waiting for all threads to end.
   * Note that thread_start() of main thread returns
   * only when no waiting pixels remained,
   * that is each pixel already served by some thread.
   */
  for (uint i = 0; i < NTHREADS - 1; ++i)
  {
    pthread_join(_threadData->threads[i], NULL);
  }

  //save the result
  if (!_threadData->outImage->WriteToFile(fileName))
  {
    cerr << "Error while writing to the output image file.\n";
    exit(RC_INPUT_ERROR);
  }
  delete _threadData;
  cout << "\rAll done." << endl;
}


void* Camera::thread_start(void* instance)
{
  Camera *camera = (Camera *) instance;
  camera->renderThread();
  return NULL;
}

void Camera::renderThread()
{
  ThreadData &data = *_threadData;
  const uint tid = threadId(); //get id of current thread

  //fraction of one chunk out of the whole image (in percents).
  const double chunkProgress =
    100.0 * double(THREAD_SERV_CHUNK) / double(data.totalPixels);

  const int w = data.outImage->TellWidth();
  ulong begin, end;
  int x, y;
  Ray ray;
  ray.O() = _position;
  Color3d C;

  bool firstChunk = true; //for sane progress reporting

  //repeat until more pixels wait for service
  while(true)
  {
    //lock the counter
    if (data.counterLock.lock()) return;
    //get another range of pixel to treat
    begin = data.nextPixel;
    end = min(begin + THREAD_SERV_CHUNK, data.totalPixels);
    data.nextPixel = end; //update unserviced range

    //report the progress
    if (firstChunk)
    { //on first time no progress yet
      firstChunk = false;
    }
    else
    { //report about done chunk
      cout << "\rDone " << uint(data.progress) << '%' << flush;
      data.progress = min(100.0, data.progress + chunkProgress);
    }
    //release the counter lock
    if (data.counterLock.unlock()) return;

    if (end <= begin) return; //all done: no more pixels to render

    //treat all pixels in the responsibility range
    for (uint i = begin; i < end; ++i)
    {
      //calculate (x,y) position of current pixel (image coordinates)
      y = i / w;
      x = i - y * w;

      //render current pixel
      C = renderPixel(x, y, ray, tid);

      //Color Range Clamping
      for (uint c = 0; c < 3; ++c)
      {
        if (C[c] > 1.0) C[c] = 1.0;
        else if (C[c] < 0.0) C[c] = 0.0;
      }

      //lock for exclusive write access on the output image
      if (data.imageLock.lock()) return;
      //set the pixel color on the output image
      (*data.outImage)(x, y)->Red = (ebmpBYTE) (C[0] * 255.0);
      (*data.outImage)(x, y)->Green = (ebmpBYTE) (C[1] * 255.0);
      (*data.outImage)(x, y)->Blue = (ebmpBYTE) (C[2] * 255.0);

      //release lock from the image
      if (data.imageLock.unlock()) return;

    } //end of rendering loop
  } //end of rendering progress loop
}

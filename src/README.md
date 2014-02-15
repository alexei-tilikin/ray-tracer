Basic Ray Tracer
############

This project builds static ray tracing library libRayTrace.a and example executable
with sample 3D scene.

It was my course project in Computer Graphics. The design was predefined.

The code written on C++ for linux. libRayTrace is multithreaded framework for 
defining and rendering static 3D scenes with the ray tracing technique.
All computations made on CPU without shaders or OpenGL scenes.

See MyScene.bmp with rendered example scene.

Dependencies:
######

* librandom from www.agner.org/random/
Included.
Precompiled library for random numbers generation.

* easyBMP from http://easybmp.sourceforge.net
Included.
Used to display the resulting image in BMP format, and to read the texture files.

* OpenMesh from http://www.openmesh.org/
Install the library on your system, before building the project.

Build:
######

Cmake used to build from source. The simpliest scenario:
    mkdir build && cd build
    cmake ..
    make

Recommended cmake options
    cmake -D CMAKE_BUILD_TYPE=Release -D NTHREADS=? ..

NTHREADS - number of threads to run concurrently. Default is 4 threads.
Do not set threads to more than number of cores on your system, or the process
will rather slow down. The optimal setting should be equal to number of CPU cores.
Valid range is [1, 10]. For invalid value, default 4 will be used.

-D CMAKE_BUILD_TYPE=?
Defines target options. May be either Release or Debug. Default is Debug.
Set Release explicitly for working example.

Usage:
######
    ./example

Usage options:
######
    ./example [--resource_path string] [--supersampling int] [--smooth_reflections int] [--smooth_reflections_cutoff float]
    [--pic_height int] [--pic_width int]
    
resource_path {string} - relative or absolute path to directory that contains directories 'models'
and 'testures' with runtime resources for the example. Defaults to '../../', as the relative path from src/build
        
supersampling {int} - number of sample rays for each pixel. Default is 1. If more than 1, then this number of samples
will be raken for each pixel, and the result will be the average between them.
This setting gives some smoothing effect mainly on the borders between objects. It's not that heavy as the following settings.
Valid range is [1, 10].
        
smooth_reflections {int} - number of recursive rays for each reflection/refraction. Default is 1.
May be used to get more smooth images and more realistic shadows.
Any numbers larger than 1 will slow down the rendering process significatly!
Valid range is [1, 10].
        
smooth_reflections_cutoff {float/double} - angle of cutoff in degrees for range of reflection/refraction rays
that will be sampled. Default is 0. Works only if smooth_reflections > 1. Defines range of smoothing.
Actual directions of sampling rays within the defined range will be random.
Valid range is [0.0, 89.0].
        
pic_height {int} - height of rendered picture in pixels. Dafults to 800. Valid range is [50, 1000].
    
pic_width {int} - width of rendered picture in pixels. Dafults to 800. Valid range is [50, 1000].
        
        
Technical abilities:
######

    Scene Objects
* Polygon - 2D object with textures support.
* 3D mesh - supported by OpenMesh library.
* Sphere (parametrized) - with textures support.
* Cylinder (parametrized) - with textures support.

    Scene setup
Defined by Camera class.
pos - camera position point.
coi - "center of interest" there the camer looks.
up - 3D vector, defines camera up.
fovH - the angle for horizontal field of view (in radians).

    Spherical light source
Class SphereLight defines spherical light with custom center position, sphere radius, and the light color.
It defines virtual sphere that illuminates any object gradually, giving smooth shades.

Another light types are AmbientLight and LocatedLight (for point light source).

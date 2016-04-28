# surfmorph
Mesh pose interpolation in C++11 with Eigen.

This is an implementation of
[as-rigid-as-possible surface morphing](http://link.springer.com/article/10.1007/s11390-011-1154-3)
for 3D meshes. It is designed as a header-only library and it only depends on
[Eigen](http://eigen.tuxfamily.org). Some bits are parallelized with
[OpenMP](http://openmp.org), although that can be disabled at compile time.

## What does it do?

It allows you to create an animation for a 3D mesh based on a sequence of two or
more key poses. Intermmediate poses can be interpolated with arbitrary precision
to generate animation frames, and the result shows an animation that is "as
rigid as possible" in the sense that in minimizes the deformation of the
triangles along the way.

In order to keep dependencies at minimum, the library uses exclusively Eigen
matrices for its operation, and it does not implement or uses any mesh structure
as such. However, integrating it with any mesh processing library should be
rather straightforward. Check the [app](app) directory to see an example that
integrates [OpenMesh](http://www.openmesh.org).

## How do I use it?

The library is quite simple and should be fairly easy to use. To install it,
just copy the content of the [include](include) directory to your project or
your system include path. Then, include the file `surfmorph/surfmorph.h` in your
source.

The library contains just one template class, `surfmorph::SurfaceMorph`, which
receives the preferred scalar type as template parameter (generally `float` or
`double`). The [source code](surfmorph/surfmorph.h) is quite commented and
describes the available operations in the class. The code below is a simple
example program demonstrating its usage.

```C++
//! Generate some triangles data and interpolate it along time.

#include <Eigen/Core>
#include <surfmorph/surfmorph.h>

#include <iostream>
#include <iomanip>
#include <vector>

int main()
{
    // Define mesh matrices types
    typedef Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> TrianglesMat;
    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> CoordsMat;
    // Define interpolator type
    typedef surfmorph::SurfaceMorph<double> SurfaceMorph;

    // Generate some data
    TrianglesMat triangles(3, 2);
    CoordsMat pose0(3, 4);
    CoordsMat pose1(3, 4);
    CoordsMat pose2(3, 4);
    // Each triangle is a vector of vertex indices
    triangles << 0, 1,
                 1, 2,
                 2, 3;
    // Each pose has the coordinates of every vertex as column vectors
    pose0 << 0.0, 1.0, 1.0, 0.0,
             0.0, 1.0, 0.0, 1.0,
             0.0, 1.0, 1.0, 1.0;
    pose1 << 0.3, 1.5, 1.7, 0.2,
             0.1, 1.2, 0.2, 1.8,
             0.2, 1.3, 1.4, 1.5;
    pose2 << 0.1, 1.2, 1.4, 0.5,
             0.8, 1.3, 0.4, 1.8,
             0.5, 1.9, 1.6, 1.3;

    // Create the interpolator
    SurfaceMorph sm(triangles, pose0);
    sm.addPose(pose1);
    sm.addPose(pose2);

    // Print the triangles matrix
    std::cout << "Triangles:" << std::endl;
    for (int i = 0; i < triangles.rows(); i++) {
        for (int j = 0; j < triangles.cols(); j++) {
            std::cout << triangles(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // Each pose is assigned a sequential integer time point; time point 0
    // is the first pose, time point 1 the second one, and time point 1.5
    // is an interpolation midway between the first and the second pose.
    std::cout << std::setprecision(2);
    for (double t = 0.0; t <= double(sm.numPoses() - 1); t += .1) {
        // Compute interpolated pose at time point
        CoordsMat interp = sm.interpolatePoseAt(t);
        // Print the pose coordinates
        std::cout << "Interpolated pose at " << t << ":" << std::endl;
        for (int i = 0; i < interp.rows(); i++) {
            for (int j = 0; j < interp.cols(); j++) {
                std::cout << interp(i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}
```

To compile this program with [GCC](https://gcc.gnu.org) on Unix, save it to a
file, say `surfmorph_test.cpp`, and run:

```Shell
$ gcc -std=gnu++11 -I<path to Eigen library> -I<path to this library> -fopenmp -o surfmorph_test surfmorph_test.cpp
```

Or, if you prefer not to use [OpenMP](http://openmp.org) (more on this
[later](#multithreading)), then run:

```Shell
$ gcc -std=gnu++11 -I<path to Eigen library> -I<path to this library> -DSURFMORPH_DONT_PARALLELIZE -o surfmorph_test surfmorph_test.cpp
```

And then test it with:

```Shell
$ ./surfmorph_test.cpp
```

## Is there some other more interesting example?

The [app](app) directory contains an application that displays an interactive
animation interpolated along the given poses. Besides this library and
[Eigen](http://eigen.tuxfamily.org/), it requires
[OpenMesh](http://www.openmesh.org), [Qt 5](https://www.qt.io) and
[libQGLViewer](http://libqglviewer.com). Also, the compilation system has only
been configure for Linux, although it should be straightforward to port to other
platforms.

## Multithreading

By default, the library uses [OpenMP](http://openmp.org) to parallelize some of
its operations. However, you can disable this by defining the macro
`SURFMORPH_DONT_PARALLELIZE` at compile time.

Pose interpolation, which is the most expensive operation, is not parallelized
by default; the reason is that, in general, interpolations are computed for a
sequence of time points, and it is more efficient to parallelize at time-point
level (that is, compute multiple interpolations in parallel). Nonetheless, it is
possible to parallelize the interpolation operation by defining the macro
`SURFMORPH_PARALLELIZE_INTERPOLATION` at compile time. Note that this is
generally not recommended unless only a single interpolation is computed.

## What are [Shape interpolation.ipynb](Shape%20interpolation.ipynb) and [surfmorph_codegen.h](include/surfmorph/surfmorph_codegen.h)?

The implementation of the algorithm requires some complex math derivation that
is not practical to solve by hand.
[Shape interpolation.ipynb](Shape%20interpolation.ipynb) is a Python notebook that
uses [SymPy](http://www.sympy.org) to compute this derivation and generate the
code that implements it in
[surfmorph_codegen.h](include/surfmorph/surfmorph_codegen.h).
[Shape interpolation.ipynb](Shape%20interpolation.ipynb) is not necessary to use
the library, but [surfmorph_codegen.h](include/surfmorph/surfmorph_codegen.h)
is, although you should not include it directly in your source code.

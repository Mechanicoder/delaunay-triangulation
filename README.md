# delaunay-triangulation

*Forked from [Bl4ckb0ne/delaunay-triangulation](https://github.com/Bl4ckb0ne/delaunay-triangulation)*

[![Build Status](https://travis-ci.com/Mechanicoder/delaunay-triangulation.svg?branch=master)](https://travis-ci.com/Mechanicoder/delaunay-triangulation)

## Pseudo-code algorithm

Pseudo-code can be found on [Wikipedia](https://en.wikipedia.org/wiki/Bowyer–Watson_algorithm)

```
function BowyerWatson (pointList)
      // pointList is a set of coordinates defining the points to be triangulated
      triangulation := empty triangle mesh data structure
      add super-triangle to triangulation // must be large enough to completely contain all the points in pointList
      for each point in pointList do // add all the points one at a time to the triangulation
         badTriangles := empty set
         for each triangle in triangulation do // first find all the triangles that are no longer valid due to the insertion
            if point is inside circumcircle of triangle
               add triangle to badTriangles
         polygon := empty set
         for each triangle in badTriangles do // find the boundary of the polygonal hole
            for each edge in triangle do
               if edge is not shared by any other triangles in badTriangles
                  add edge to polygon
         for each triangle in badTriangles do // remove them from the data structure
            remove triangle from triangulation
         for each edge in polygon do // re-triangulate the polygonal hole
            newTri := form a triangle from edge to point
            add newTri to triangulation
      for each triangle in triangulation // done inserting points, now clean up
         if triangle contains a vertex from original super-triangle
            remove triangle from triangulation
      return triangulation
```

## Sample

![alt text](https://github.com/Mechanicoder/delaunay-triangulation/blob/master/pic/sample.png "Sample image (if you see this, then the image can't load or hasn't loaded yet)")


From the [Wikipedia page of the algorithm](https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm)

###Other test cases

*case 1*

	points = { { 0.2, 0.2 }, { 0.2, 0.8 }, { 0.2, 1.2 }, { 0.8, 1.2 } };
	//points[1].x = 0.212151; // concave
	//points[1].x = 0.212152; // convex

![case 1](https://github.com/Mechanicoder/delaunay-triangulation/blob/master/pics/test_case_convex-concave.png)

*case 2*

	// Generate self-intersection triangles if treat <onside> & <inside> as inside of <CircumCircle>
    // And works fine if treat <onside>  as outside of <circumCircle>
	points = {
	    { 1.0000000000000000, 0.00000000000000000 },
	    { 0.92387953251128674, 0.38268343236508978 },
	    { 0.70710678118654757, 0.70710678118654757 },
	    { 0.38268343236508984, 0.92387953251128674 },
	    { 6.1232339957367660e-017, 1.0000000000000000 } };

	// works fine after the following operation
    //std::reverse(points.begin(), points.end());

    // works fine for following points
    //points = {
    //    { 1, 0 }, { 0.92388, 0.382683 }, { 0.707107, 0.707107 },
    //    { 0.382683, 0.92388 }, { 0, 1 } }; // fine

![case 2](https://github.com/Mechanicoder/delaunay-triangulation/blob/master/pics/test_case_self_intersection.png)

*case 3*

	// Cannot generate any triangle with the following 4 points 
    points = {
        { 0.6025588633159, 0.2 },
        { 0.5877586727660, 0.7 },
        { 0.5899647831820, 0.5 },
        { 0.6078756525049, 0 } };

	// works fine after this point inserted
    //points.emplace_back(Pnt(0.6025588633159, -0.2));

![case 3](https://github.com/Mechanicoder/delaunay-triangulation/blob/master/pics/test_case_failed.png)


## Requirement

You will need [SFML 2+](http://www.sfml-dev.org/download/sfml/2.3.2/) to run the example, and C++11 to compile it.

## Usage

To build it, you can type in :
```sh
make
```
You may change the compiler on the makefile (using the CXX var)
```sh
make CXX=g++            # to use the GCC compiler
make CXX=clang++        # default compiler
```

The executable name is ``` delaunay ```, without arguments
```sh
./delaunay
```

Number of points to use to create triangulation can be given
```sh
./delaunay 50
```

You can compile and run tests with the following commands
```sh
make test
./tests_delaunay
```


You also can clear the executable and the build folder.
```sh
make clean
```

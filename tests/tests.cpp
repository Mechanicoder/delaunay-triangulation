#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "delaunay.h"

TEST_CASE("Delaunay triangulation should be able to triangulate 3 points as float", "[DelaunayTest]") {
    std::vector<Vector2<float> > points;
    points.push_back(Vector2<float>(0.0f, 0.0f));
    points.push_back(Vector2<float>(1.0f, 0.0f));
    points.push_back(Vector2<float>(0.0f, 1.0f));
    Delaunay<float> triangulation;
    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);
    REQUIRE(1 == triangles.size());
    // const std::vector<Edge<float> > edges = triangulation.getEdges();
}

TEST_CASE( "Delaunay triangulation should be able to triangulate 3 points as double", "[DelaunayTest]" ) {
    std::vector<Vector2<double> > points;
    points.push_back(Vector2<double>(0.0, 0.0));
    points.push_back(Vector2<double>(1.0, 0.0));
    points.push_back(Vector2<double>(0.0, 1.0));
    Delaunay<double> triangulation;
    const std::vector<Triangle<double> > triangles = triangulation.triangulate(points);
    REQUIRE(1 == triangles.size());
    // const std::vector<Edge<double> > edges = triangulation.getEdges();
}

TEST_CASE("Delaunay triangulation should be able to handle duplicated 3 points as double", "[DelaunayTest]") {
    std::vector<Vector2<double> > points;
    points.push_back(Vector2<double>(0.0, 0.0));
    points.push_back(Vector2<double>(0.0, 0.0));
    points.push_back(Vector2<double>(1.0, 0.0));
    points.push_back(Vector2<double>(0.0, 1.0));
    Delaunay<double> triangulation;
    const std::vector<Triangle<double> > triangles = triangulation.triangulate(points);
    REQUIRE(1 == triangles.size());
    // const std::vector<Edge<double> > edges = triangulation.getEdges();
}

TEST_CASE("Delaunay triangulation should be able to handle 10000 points as float", "[DelaunayTest]") {
    std::vector<Vector2<float> > points(1e4);
    srand(666);
    for (size_t i=0; i < 1e4; ++i)
    {
        float x = (float)rand() / (float)RAND_MAX;
        float y = (double)rand() / (float)RAND_MAX;
        points.at(i) = Vector2<float>(x, y);
    }
    REQUIRE(10000 == points.size());
    Delaunay<float> triangulation;
    const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);
}

TEST_CASE("Delaunay triangulation should be able to handle 10000 points as double", "[DelaunayTest]") {
    std::vector<Vector2<double> > points(1e4);
    srand(666);
    for (size_t i=0; i < 1e4; ++i)
    {
        double x = (double)rand() / (double)RAND_MAX;
        double y = (double)rand() / (double)RAND_MAX;
        points.at(i) = Vector2<double>(x, y);
    }
    REQUIRE(10000 == points.size());
    Delaunay<double> triangulation;
    const std::vector<Triangle<double> > triangles = triangulation.triangulate(points);
}

// failed test
TEST_CASE("Delaunay triangulation Can Not get a convex boundary", "[DelaunayTest]") {
    std::vector<Vector2<double>> points = {
        { 0.2, 0.2 }, { 0.2, 0.8 }, { 0.2, 1.2 }, { 0.8, 1.2 } };
    points[1].x = 0.212151; // concave
    //points[1].x = 0.212152; // convex
    Delaunay<double> triangulation;
    const std::vector<Triangle<double> > triangles = triangulation.triangulate(points);
}

TEST_CASE("Delaunay triangulation Will generate self-intersection triangles", "[DelaunayTest]") {
    std::vector<Vector2<double>> points{
        { 1.0000000000000000, 0.00000000000000000 },
        { 0.92387953251128674, 0.38268343236508978 },
        { 0.70710678118654757, 0.70710678118654757 },
        { 0.38268343236508984, 0.92387953251128674 },
        { 6.1232339957367660e-017, 1.0000000000000000 } }; // self-intersect
    //std::reverse(points.begin(), points.end()); // would be successfule
    
    //points = {
    //    { 1, 0 }, { 0.92388, 0.382683 }, { 0.707107, 0.707107 }, 
    //    { 0.382683, 0.92388 }, { 0, 1 } }; // would be successful
    Delaunay<double> triangulation;
    const std::vector<Triangle<double> > triangles = triangulation.triangulate(points);
}

TEST_CASE("Delaunay triangulation Can Not generate triangles with 4 points", "[DelaunayTest]") {
    std::vector<Vector2<double>> points = {
        { 0.6025588633159, 0.2 },
        { 0.5877586727660, 0.7 },
        { 0.5899647831820, 0.5 },
        { 0.6078756525049, 0 } };

    // it would generate 3 triangles after this point inserted.
    // points.emplace_back(Pnt(0.6025588633159, -0.2));

    Delaunay<double> triangulation;
    const std::vector<Triangle<double> > triangles = triangulation.triangulate(points);
    REQUIRE(0 == triangles.size());
}


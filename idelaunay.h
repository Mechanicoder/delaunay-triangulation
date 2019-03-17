#pragma once
#include <algorithm>
#include <vector>

#ifdef USE_VECTOR2
#include "vector2.h"
typedef Vector2<double> Pnt;
#else
#ifdef USE_OCCT_PNT
#include <gp_Pnt2d.hxx>
#endif
class Pnt
{
public:
    Pnt() :x(0), y(0){}

    Pnt(double _x, double _y) : x(_x), y(_y){}

    Pnt(const Pnt &v) : x(v.x), y(v.y){}

    //
    // Operations
    //
    double dist2(const Pnt &v) const
    {
        double dx = x - v.x;
        double dy = y - v.y;
        return dx * dx + dy * dy;
    }

    double dist(const Pnt &v) const
    {
        return std::sqrt(dist2(v));
    }

    double norm2() const
    {
        return x * x + y * y;
    }
#ifdef USE_OCCT_PNT
    Pnt(const gp_Pnt2d &occ_pnt) : x(occ_pnt.X()), y(occ_pnt.Y()) {}

    operator gp_Pnt2d() const
    {
        return gp_Pnt2d(x, y);
    }
#endif

    double x;
    double y;
};
#endif

class IdxEdge
{
public:
    static size_t EdgeCnt;

    size_t ipnt1, ipnt2;
    size_t edgenum;
    //size_t itri1, itri2;

    bool isbad;

    IdxEdge(size_t _1, size_t _2)
        : ipnt1(_1), ipnt2(_2), edgenum(EdgeCnt++), isbad(false) {}
    IdxEdge(){}
};

class IdxTriangle
{
public:
    static size_t TriCnt;

    size_t ipnt1, ipnt2, ipnt3;
    size_t trinum;
    
    //size_t iedge1, iedge2, iedge3;

    bool isbad;

    IdxTriangle(size_t _1 = 0, size_t _2 = 0, size_t _3 = 0)
        : ipnt1(_1), ipnt2(_2), ipnt3(_3), trinum(TriCnt++), isbad(false) { }
};

class IDelaunay
{
private:
    std::vector<Pnt> _vertices;

    std::vector<IdxEdge> _edges;
    std::vector<IdxTriangle> _triangles;

public:
    // Convex boundaries are not guaranteed
    void Triangulate(const std::vector<Pnt> &vertices);

    // With given Convex-Hull
    void Triangulate(const std::vector<Pnt> &vertices, const std::vector<size_t> &convex_hull);

    const std::vector<IdxEdge>& GetEdges() const { return _edges; }
    const std::vector<IdxTriangle>& GetTriangles() const { return _triangles; }

    const Pnt &GetPnt(size_t idx) const { return _vertices[idx]; }

private:
    bool OnDiffSideOfEdge(const Pnt &e1, const Pnt &e2, const Pnt &p1, const Pnt &p2) const;
};


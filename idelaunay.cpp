#include "idelaunay.h"

#include <tuple>
#include <cassert>
#include <set>

#define EQUAL_DIST 1e-7

template <class T, class C>
std::pair<size_t, size_t> FindMinMax(const std::vector<T> &data, C _Calc)
{
    if (data.size() <= 1)
    {
        return std::make_pair(0, 0);
    }

    size_t idx_min = 0, idx_max = 0;
    auto vmin = _Calc(data[idx_min]);
    auto vmax = _Calc(data[idx_max]);

    for (size_t i = 1; i < data.size(); i++)
    {
        auto v = _Calc(data[i]);
        if (v < vmin)
        {
            vmin = v;
            idx_min = i;
        }

        if (v > vmax)
        {
            vmax = v;
            idx_max = i;
        }
    }
    return std::make_pair(idx_min, idx_max);
}

size_t IdxEdge::EdgeCnt = 0;
size_t IdxTriangle::TriCnt = 0;

// return value: 0--outside, 1-onside, 2-inside
int DoesTriCircumCircleContains(const Pnt &p1, const Pnt &p2, const Pnt &p3, const Pnt &p)
{
    const double ab = p1.norm2();
    const double cd = p2.norm2();
    const double ef = p3.norm2();

    const double circum_x = (ab * (p3.y - p2.y) + cd * (p1.y - p3.y) + ef * (p2.y - p1.y)) / (p1.x * (p3.y - p2.y) + p2.x * (p1.y - p3.y) + p3.x * (p2.y - p1.y));
    const double circum_y = (ab * (p3.x - p2.x) + cd * (p1.x - p3.x) + ef * (p2.x - p1.x)) / (p1.y * (p3.x - p2.x) + p2.y * (p1.x - p3.x) + p3.y * (p2.x - p1.x));

    const Pnt circum(0.5 * circum_x, 0.5 * circum_y);
    const double circum_radius = p1.dist2(circum);
    const double dist = p.dist2(circum);
    if (dist > circum_radius) // outside
    {
        return 0;
    }
    else if (dist < circum_radius - EQUAL_DIST) // inside
    {
        return 2;
    }
    else
    {
        assert(circum_radius - dist < EQUAL_DIST);
        return 1;
    }
}

void IDelaunay::Triangulate(const std::vector<Pnt> &vertices)
{
    _vertices.clear();
    _edges.clear();
    _triangles.clear();

    // Store the vertices locally
    _vertices = vertices;

    // Determinate the super triangle
    size_t iminx, iminy, imaxx, imaxy;
    std::tie(iminx, imaxx) = FindMinMax(vertices, [](const Pnt &p) {return p.x; });
    std::tie(iminy, imaxy) = FindMinMax(vertices, [](const Pnt &p) {return p.y; });

    double minX = vertices[iminx].x, maxX = vertices[imaxx].x;
    double minY = vertices[iminy].y, maxY = vertices[imaxy].y;

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double deltaMax = std::max(dx, dy);
    const double midx = 0.5 * (minX + maxX);
    const double midy = 0.5 * (minY + maxY);

    const Pnt p1(midx - 20 * deltaMax, midy - deltaMax);
    const Pnt p2(midx, midy + 20 * deltaMax);
    const Pnt p3(midx + 20 * deltaMax, midy - deltaMax);

    const size_t ori_size = _vertices.size();
    _vertices.emplace_back(p1);
    _vertices.emplace_back(p2);
    _vertices.emplace_back(p3);

    _triangles.emplace_back(IdxTriangle(ori_size + 0, ori_size + 1, ori_size + 2));

    //std::cout << "Super triangle " << std::endl << Triangle(p1, p2, p3) << std::endl;

    for (size_t i_vertex = 0; i_vertex < _vertices.size() - 3; ++i_vertex)
    {
        const Pnt &p = _vertices[i_vertex];
        
        std::vector<IdxEdge> polygon;
        for (size_t i_tri = 0; i_tri < _triangles.size(); ++i_tri)
        {
            IdxTriangle &idxtri = _triangles[i_tri];
            const Pnt &tp1 = _vertices[idxtri.ipnt1];
            const Pnt &tp2 = _vertices[idxtri.ipnt2];
            const Pnt &tp3 = _vertices[idxtri.ipnt3];

            //std::cout << "Processing " << std::endl << *t << std::endl;
            
            int contain = DoesTriCircumCircleContains(tp1, tp2, tp3, p);
            // treat onside circle point as outside one
            if (contain == 2) // "Inside"
            {
                //std::cout << "Pushing bad triangle " << *t << std::endl;
                idxtri.isbad = true;

                polygon.emplace_back(IdxEdge(idxtri.ipnt1, idxtri.ipnt2));
                polygon.emplace_back(IdxEdge(idxtri.ipnt1, idxtri.ipnt3));
                polygon.emplace_back(IdxEdge(idxtri.ipnt2, idxtri.ipnt3));
            }
            else
            {
                //std::cout << " does not contains " << *p << " in his circum center" << std::endl;
            }
        }

        _triangles.erase(std::remove_if(begin(_triangles), end(_triangles), 
            [](IdxTriangle &t) { return t.isbad; }), end(_triangles));

        for (size_t i_poly1 = 0; i_poly1 < polygon.size(); ++i_poly1)
        {
            for (size_t i_poly2 = i_poly1 + 1; i_poly2 < polygon.size(); ++i_poly2)
            {
                bool equal1 = polygon[i_poly1].ipnt1 == polygon[i_poly2].ipnt1 &&
                    polygon[i_poly1].ipnt2 == polygon[i_poly2].ipnt2;
                bool equal2 = polygon[i_poly1].ipnt2 == polygon[i_poly2].ipnt1 &&
                    polygon[i_poly1].ipnt1 == polygon[i_poly2].ipnt2;
                if (equal1 || equal2)
                {
                    polygon[i_poly1].isbad = true;
                    polygon[i_poly2].isbad = true;
                }
            }
        }

        polygon.erase(std::remove_if(begin(polygon), end(polygon), [](const IdxEdge &e){
            return e.isbad;
        }), end(polygon));

        for (size_t i_poly = 0; i_poly < polygon.size(); ++i_poly)
        {
            _triangles.emplace_back(IdxTriangle(polygon[i_poly].ipnt1, polygon[i_poly].ipnt2, i_vertex));
        }

        // debug
        //std::cout << "\nInsert: " << i_vertex << "\n";
        //for (size_t i = 0; i < _triangles.size(); i++)
        //{
        //    std::cout << _triangles[i].ipnt1 << '-'
        //        << _triangles[i].ipnt2 << '-'
        //        << _triangles[i].ipnt3 << '\t';
        //}
        //std::cout << std::endl << std::endl;
    }

    _triangles.erase(std::remove_if(begin(_triangles), end(_triangles),
        [ori_size](IdxTriangle &t) 
    { 
        return t.ipnt1 >= ori_size || t.ipnt2 >= ori_size || t.ipnt3 >= ori_size; 
    }), end(_triangles));

    _vertices.erase(_vertices.end() - 3, _vertices.end());

    _edges.clear();
    for (size_t i_tri = 0; i_tri < _triangles.size(); ++i_tri)
    {
        _edges.emplace_back(IdxEdge(_triangles[i_tri].ipnt1, _triangles[i_tri].ipnt2));
        _edges.emplace_back(IdxEdge(_triangles[i_tri].ipnt2, _triangles[i_tri].ipnt3));
        _edges.emplace_back(IdxEdge(_triangles[i_tri].ipnt1, _triangles[i_tri].ipnt3));
    }
}

// triangulate with convex hull indices
void IDelaunay::Triangulate(const std::vector<Pnt> &vertices,
    const std::vector<size_t> &convex_hull)
{
    std::cout << "IDelaunay with convex hull\n";

    std::vector<Pnt> cv_pnts(convex_hull.size());
    for (size_t i = 0; i < convex_hull.size(); i++)
    {
        cv_pnts[i] = vertices[convex_hull[i]];
    }
    IDelaunay cv_mesh;
    cv_mesh.Triangulate(cv_pnts);
    const std::vector<IdxTriangle> &tris = cv_mesh.GetTriangles();

    _vertices.clear();
    _edges.clear();
    _triangles.clear();
    _vertices = vertices;

    for (size_t i = 0; i < tris.size(); ++i)
    {
        _triangles.emplace_back(IdxTriangle(
            convex_hull[tris[i].ipnt1], convex_hull[tris[i].ipnt2], convex_hull[tris[i].ipnt3]));
    }

    // skip convex points, <index, is_boundary>
    std::vector<std::pair<size_t, bool>> map(_vertices.size());
    // find first Pnt not in convex_hull
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        map[i].first = i;
        map[i].second = false;
        for (size_t j = 0; j < convex_hull.size(); ++j)
        {
            if (i == convex_hull[j])
            {
                map[i].second = true;
                break;
            }
        }
    }

    // Create a list of triangles, and add the super triangle and convex triangle in it
    for (size_t i_map = 0; i_map < map.size(); ++i_map)
    {
        size_t i_vertex = map[i_map].first;
        if (map[i_map].second)
        {
            continue; // processed
        }
        const Pnt &p = _vertices[i_vertex];
        std::vector<IdxEdge> polygon;
        for (size_t i_tri = 0; i_tri < _triangles.size(); ++i_tri)
        {
            IdxTriangle &idxtri = _triangles[i_tri];
            const Pnt &tp1 = _vertices[idxtri.ipnt1];
            const Pnt &tp2 = _vertices[idxtri.ipnt2];
            const Pnt &tp3 = _vertices[idxtri.ipnt3];

            int contain = DoesTriCircumCircleContains(tp1, tp2, tp3, p);
            if (contain == 2)
            {
                //std::cout << "Pushing bad triangle " << *t << std::endl;
                idxtri.isbad = true;

                polygon.emplace_back(IdxEdge(idxtri.ipnt1, idxtri.ipnt2));
                polygon.emplace_back(IdxEdge(idxtri.ipnt1, idxtri.ipnt3));
                polygon.emplace_back(IdxEdge(idxtri.ipnt2, idxtri.ipnt3));
            }
            else
            {
                //std::cout << " does not contains " << *p << " in his circum center" << std::endl;
            }
        }

        // todo optimize
        _triangles.erase(std::remove_if(begin(_triangles), end(_triangles),
            [](IdxTriangle &t) { return t.isbad; }), end(_triangles));

        for (size_t i_poly1 = 0; i_poly1 < polygon.size(); ++i_poly1)
        {
            for (size_t i_poly2 = i_poly1 + 1; i_poly2 < polygon.size(); ++i_poly2)
            {
                bool equal1 = polygon[i_poly1].ipnt1 == polygon[i_poly2].ipnt1 &&
                    polygon[i_poly1].ipnt2 == polygon[i_poly2].ipnt2;
                bool equal2 = polygon[i_poly1].ipnt2 == polygon[i_poly2].ipnt1 &&
                    polygon[i_poly1].ipnt1 == polygon[i_poly2].ipnt2;
                if (equal1 || equal2)
                {
                    polygon[i_poly1].isbad = true;
                    polygon[i_poly2].isbad = true;
                }
            }
        }

        polygon.erase(std::remove_if(begin(polygon), end(polygon), [](const IdxEdge &e){
            return e.isbad;
        }), end(polygon));

        for (size_t i_poly = 0; i_poly < polygon.size(); ++i_poly)
        {
            _triangles.emplace_back(IdxTriangle(polygon[i_poly].ipnt1, polygon[i_poly].ipnt2, i_vertex));
        }
    }

    _edges.clear();
    for (size_t i_tri = 0; i_tri < _triangles.size(); ++i_tri)
    {
        _edges.emplace_back(IdxEdge(_triangles[i_tri].ipnt1, _triangles[i_tri].ipnt2));
        _edges.emplace_back(IdxEdge(_triangles[i_tri].ipnt2, _triangles[i_tri].ipnt3));
        _edges.emplace_back(IdxEdge(_triangles[i_tri].ipnt1, _triangles[i_tri].ipnt3));
    }
}

bool IDelaunay::OnDiffSideOfEdge(const Pnt &e1, const Pnt &e2, const Pnt &p1, const Pnt &p2) const
{
    double e1e2x = e2.x - e1.x;
    double e1e2y = e2.y - e1.y;
    double e2p1x = p1.x - e2.x;
    double e2p1y = p1.y - e2.y;
    double e2p2x = p2.x - e2.x;
    double e2p2y = p2.y - e2.y;

    return (e1e2x * e2p1y - e2p1x * e1e2y) * (e1e2x * e2p2y - e2p2x * e1e2y) < 0.0;
}

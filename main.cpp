#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <array>

#include <SFML/Graphics.hpp>

#include "vector2.h"
#include "triangle.h"
#include "delaunay.h"

#include "idelaunay.h"

#define N 9000

#define WIDTH 500
#define HEIGHT 500
#define SHRINK 300

float RandomFloat(float a, float b) {
    const float random = ((float) rand()) / (float) RAND_MAX;
    const float diff = b - a;
    const float r = random * diff;
    return a + r;
}

void Draw(const std::vector<std::pair<Pnt, Pnt>> &edges)
{
    // SFML window
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Delaunay triangulation");

    // Transform each points of each vector as a rectangle
    //std::vector<sf::RectangleShape*> squares;
    //for (const auto p : points) {
    //    sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(4, 4));
    //    c1->setPosition(p.x, p.y);
    //    squares.push_back(c1);
    //}

    // Make the lines
    std::vector<std::array<sf::Vertex, 2> > lines;
    for (const auto &e : edges)
    {
        const Pnt &p1 = e.first;
        const Pnt &p2 = e.second;
        lines.push_back({ {
                sf::Vertex(sf::Vector2f((float)p1.x * SHRINK + 2, (float)p1.y * SHRINK + 2)),
                sf::Vertex(sf::Vector2f((float)p2.x * SHRINK + 2, (float)p2.y * SHRINK + 2))
            } });
    }

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        // Draw the squares
        //for (const auto &s : squares) {
        //    window.draw(*s);
        //}

        // Draw the lines
        for (const auto &l : lines) {
            window.draw(l.data(), 2, sf::Lines);
        }

        window.display();
    }
}

void TestOriginal(std::vector<Pnt> &points)
{
    std::clock_t now = std::clock();
    Delaunay<double> delaunay;
    delaunay.triangulate(points);
    std::cout << "Original: Time Consuming: \t" << std::clock() - now << "\n\n";
    
    const std::vector<Triangle<double> > triangles = delaunay.getTriangles();
    std::cout << triangles.size() << " triangles generated\n";
    const std::vector<Edge<double> > edges = delaunay.getEdges();

    std::vector<std::pair<Pnt, Pnt>> lines(edges.size());
    for (size_t i = 0; i < edges.size(); i++)
    {
        lines[i].first = edges[i].p1;
        lines[i].second = edges[i].p2;
    }
    Draw(lines);
}

void TestIDelaunay(std::vector<Pnt> &points)
{
    std::clock_t now = std::clock();
    IDelaunay mesh;
    mesh.Triangulate(points);
    std::cout << "IDelaunay: Time Consuming: \t" << std::clock() - now << "\n\n";

    const std::vector<IdxTriangle> &triangles = mesh.GetTriangles();

    std::cout << triangles.size() << " triangles generated\n";
    const std::vector<IdxEdge> &edges = mesh.GetEdges();

    std::cout << " ========= ";

    std::cout << "\nPoints : " << points.size() << std::endl;
    //for(const auto &p : points)
    //	std::cout << p << std::endl;

    std::cout << "\nTriangles : " << triangles.size() << std::endl;
    //for(const auto &t : triangles)
    //	std::cout << t << std::endl;

    std::cout << "\nEdges : " << edges.size() << std::endl;
    //for(const auto &e : edges)
    //	std::cout << e << std::endl;

    std::vector<std::pair<Pnt, Pnt>> lines(edges.size());
    for (size_t i = 0; i < edges.size(); i++)
    {
        lines[i].first = points[edges[i].ipnt1];
        lines[i].second = points[edges[i].ipnt2];
    }
    Draw(lines);
}

void Test()
{
    std::vector<Pnt> points;
    int test = 1;
    if (test == 0)
    {
        points.resize(N);
        srand(time(NULL));
        for (size_t i = 0; i < N; ++i)
        {
            points.at(i) = Pnt(RandomFloat(0, WIDTH), RandomFloat(0, HEIGHT));
        }
    }
    else if (test == 1)
    {
        points = {
            { 0.2, 0.2 }, { 0.2, 0.8 }, /*{ 0.8, 0.2 },*/{ 0.2, 1.2 }, { 0.8, 1.2 } };
        points[1].x = 0.212151; // concave
        //points[1].x = 0.212152; // convex

        bool sort = true;
        if (sort)
        {
            std::sort(points.begin(), points.end(), [](Pnt &p1, Pnt &p2)
            {
                if (p1.x == p2.x)
                {
                    return p1.y < p2.y;
                }
                else
                {
                    return p1.x < p2.x;
                }
            });
        }
    }
    else if (test == 2)
    {
        // Generate self-intersection triangles because of <Onside Inside Outside> judgement
        points = {
            { 1.0000000000000000, 0.00000000000000000 },
            { 0.92387953251128674, 0.38268343236508978 },
            { 0.70710678118654757, 0.70710678118654757 },
            { 0.38268343236508984, 0.92387953251128674 },
            { 6.1232339957367660e-017, 1.0000000000000000 } };
        
        // works fine after the following operation
        //std::reverse(points.begin(), points.end());

        // works fine 
        //points = {
        //    { 1, 0 }, { 0.92388, 0.382683 }, { 0.707107, 0.707107 },
        //    { 0.382683, 0.92388 }, { 0, 1 } }; // fine
    }
    else if (test == 3)
    {
        // Cannot generate any triangles with the following 4 points 
        points = {
            { 0.6025588633159, 0.2 },
            { 0.5877586727660, 0.7 },
            { 0.5899647831820, 0.5 },
            { 0.6078756525049, 0 } };

        // works fine after this point inserted
        points.emplace_back(Pnt(0.6025588633159, -0.2));

        std::vector<std::pair<Pnt, Pnt>> lines(points.size());
        for (size_t i = 0, j = lines.size() - 1; i < lines.size(); j = i++)
        {
            lines[i].first = points[i];
            lines[i].second = points[j];
        }
        Draw(lines);
    }

    TestIDelaunay(points);
}

int main(int argc, char * argv[])
{
    Test();
    return 0;

	int numberPoints = 400;
	if (argc==1)
	{
		numberPoints = (int) roundf(RandomFloat(4, numberPoints));
	}
	else if (argc>1)
	{
		numberPoints = atoi(argv[1]);
	}
    numberPoints = N;

	srand (time(NULL));

	std::cout << "Generating " << numberPoints << " random points" << std::endl;

	std::vector<Vector2<float> > points;
	for(int i = 0; i < numberPoints; ++i) {
		points.push_back(Vector2<float>(RandomFloat(0, 800), RandomFloat(0, 600)));
	}

	Delaunay<float> triangulation;
	const std::vector<Triangle<float> > triangles = triangulation.triangulate(points);
	std::cout << triangles.size() << " triangles generated\n";
	const std::vector<Edge<float> > edges = triangulation.getEdges();

	std::cout << " ========= ";

	std::cout << "\nPoints : " << points.size() << std::endl;
	//for(const auto &p : points)
	//	std::cout << p << std::endl;

	std::cout << "\nTriangles : " << triangles.size() << std::endl;
	//for(const auto &t : triangles)
	//	std::cout << t << std::endl;

	std::cout << "\nEdges : " << edges.size() << std::endl;
	//for(const auto &e : edges)
	//	std::cout << e << std::endl;

	// SFML window
	sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Delaunay triangulation");

	// Transform each points of each vector as a rectangle
	std::vector<sf::RectangleShape*> squares;

	for(const auto p : points) {
		sf::RectangleShape *c1 = new sf::RectangleShape(sf::Vector2f(4, 4));
		c1->setPosition(p.x, p.y);
		squares.push_back(c1);
	}

	// Make the lines
	std::vector<std::array<sf::Vertex, 2> > lines;
	for(const auto &e : edges) {
		lines.push_back({{
			sf::Vertex(sf::Vector2f(e.p1.x + 2, e.p1.y + 2)),
			sf::Vertex(sf::Vector2f(e.p2.x + 2, e.p2.y + 2))
		}});
	}

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}

		window.clear();

		// Draw the squares
		for(const auto &s : squares) {
			window.draw(*s);
		}

		// Draw the lines
		for(const auto &l : lines) {
			window.draw(l.data(), 2, sf::Lines);
		}

		window.display();
	}

	return 0;
}

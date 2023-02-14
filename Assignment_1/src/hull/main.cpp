////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	// TODO
	return 0;
}

struct Compare {
	Point p0; // Lowest point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		// TODO
		return (std::arg(p1 - p0) < std::arg(p2 - p0));
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	// TODO
	double val = (b.imag() - a.imag()) * (c.real() - b.real()) - (c.imag() - b.imag()) * (b.real() - a.real());
	return (val <= 0)? true: false;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	// TODO
	Point lowest_p = points[0];
	for (int i = 0; i < points.size(); i++){
		if ((points[i].imag() < lowest_p.imag()) || 
			(points[i].imag() == lowest_p.imag() && points[i].real() < lowest_p.real())) {
			lowest_p = points[i];
		}
	}
	order.p0 = lowest_p;
	std::sort(points.begin(), points.end(), order);
	Polygon hull;
	// TODO
	// use salientAngle(a, b, c) here
	for (int i = 0; i < points.size(); i++){
		
		while (hull.size() > 1 && !salientAngle(hull.end()[-2], hull.end()[-1], points[i])) {
			hull.pop_back();
		}
		hull.push_back(points[i]);
		
	}
	return hull;	
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	// TODO
	double x, y;
	int z, size;

	in >> size;
	for (int i = 0; i < size; i++) {
		in >> x >> y >> z;
		const Point point = Point(x, y);
		points.push_back(point);
	}
	in.close();
	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}

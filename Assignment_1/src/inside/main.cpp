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

double inline det(const Point &p1, const Point &p2, const Point &p3) {
	// TODO
	return (p3.imag()-p1.imag())*(p2.real()-p1.real()) - ((p2.imag()-p1.imag())*(p3.real()-p1.real()));
}


// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d) {
	// TODO
	return ((det(a, c, d)*det(b, c, d) < 0) && (det(a, b, c)*det(a, b, d) < 0));
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	// TODO
	Point outside(0, 0);
	double x_edge = 0;
	for (int i = 0; i < poly.size(); i++) {
		if (poly[i].real() > x_edge) {
			x_edge = poly[i].real();
		}
	}
	outside.real(abs(x_edge*2));
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	// TODO
	int intersect = 0;
	for (int i = 0; i < poly.size(); i++){
		if (i == poly.size() - 1) {
			if (intersect_segment(query, outside, poly[i], poly[0])) intersect ++;
		} else {
			if (intersect_segment(query, outside, poly[i], poly[i+1])) intersect ++;
		}
	}
	return (intersect % 2 != 0);
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


Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	// TODO
	Polygon polygon;
	Polygon sorted_polygon;
	std::vector<int> index;
	char header;
	while (!in.eof()) {
		in.get(header);
		if (header == 'v') {
			in.ignore();
			double x, y;
			int z;
			in >> x >> y >> z;
			Point p = Point(x, y);
			polygon.push_back(p);
		} else if (header == 'f') {
			while (!in.eof()) {
				int i;
				in >> i;
				index.push_back(i);
			} 
		} else continue;
	}
	in.close();
	sorted_polygon.resize(polygon.size());
	for (int i = 0; i < polygon.size(); i++) {
		sorted_polygon[index[i]-1] = polygon[i];
	}
	return sorted_polygon;
}


void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	// TODO
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (int i = 0; i < points.size(); i++) {
		out << points[i].real() << ' ' << points[i].imag() << " 0\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}

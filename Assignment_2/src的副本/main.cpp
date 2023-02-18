// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	Vector3d sphere_origin(0, 0, 0);
	const double sphere_radius = 0.9;

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			const double a = ray_direction.squaredNorm();
			const double b = (ray_origin-sphere_origin).dot(2*ray_direction);
			const double c = (ray_origin-sphere_origin).squaredNorm() - pow(sphere_radius, 2);
			const double d = pow(b, 2) - 4*a*c;

			if (d >= 0) {
				// The ray hit the sphere, compute the exact intersection point
				const double t = (-b-sqrt(d))/(2*a);
				Vector3d ray_intersection = ray_origin + t*ray_direction;

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,3);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.8, -0.8, -1);
	Vector3d pgram_u(1.2, 0.4, 0.5);
	Vector3d pgram_v(0.4, 1.2, 0.5);
 
	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);
			// TODO: Check if the ray intersects with the parallelogram
			Vector3d RHS(ray_origin-pgram_origin);
			Matrix3d LHS;
			LHS << pgram_u, pgram_v, -ray_direction;
			Vector3d x = LHS.colPivHouseholderQr().solve(RHS);
			double u, v, t;
			u = x(0);
			v = x(1);
			t = x(2);
			if (u>=0 && u<=1 && v>=0 && v<=1 && t>0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection(ray_origin + t*ray_direction);

				// TODO: Compute normal at the intersection point
				Vector3d intersect_u = pgram_u;
				Vector3d intersect_v = pgram_v;
				Vector3d ray_normal = (intersect_u.cross(intersect_v)).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}
	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
	
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename1("parallelogram_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,3);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.8, -0.8, -1);
	Vector3d pgram_u(1.2, 0.4, 0.5);
	Vector3d pgram_v(0.4, 1.2, 0.5);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// TODO: Prepare the ray (origin point and direction)
			Vector3d ray_origin = origin;
			Vector3d ray_direction = Vector3d(origin(0), origin(1), 0) + double(i)*x_displacement + double(j)*y_displacement - ray_origin;
			
			// TODO: Check if the ray intersects with the parallelogram
			Vector3d RHS(ray_origin-pgram_origin);
			Matrix3d LHS;
			LHS << pgram_u, pgram_v, -ray_direction;
			Vector3d x = LHS.colPivHouseholderQr().solve(RHS);
			double u, v, t;
			u = x(0);
			v = x(1);
			t = x(2);
			if (u>=0 && u<=1 && v>=0 && v<=1 && t>0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection(ray_origin + t*ray_direction);

				// TODO: Compute normal at the intersection point
				Vector3d intersect_u = pgram_u;
				Vector3d intersect_v = pgram_v;
				Vector3d ray_normal = (intersect_u.cross(intersect_v)).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename1);

	std::cout << "Simple ray tracer, one sphere with perspective projection" << std::endl;

	const std::string filename2("sphere_perspective.png");
	C = MatrixXd::Zero(800,800); // Store the color
	A = MatrixXd::Zero(800,800); // Store the alpha mask

	Vector3d sphere_origin(0, 0, 0);
	const double sphere_radius = 0.7;

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// TODO: Prepare the ray (origin point and direction)
			Vector3d ray_origin = origin;
			Vector3d ray_direction = Vector3d(origin(0), origin(1), 0) + double(i)*x_displacement + double(j)*y_displacement - ray_origin;

			// Intersect with the sphere
			const double a = ray_direction.squaredNorm();
			const double b = 2*ray_direction.dot(ray_origin-sphere_origin);
			const double c = (ray_origin-sphere_origin).squaredNorm() - pow(sphere_radius, 2);
			const double d = pow(b, 2) - 4*a*c;

			if (d >= 0) {
				// The ray hit the sphere, compute the exact intersection point
				const double t = (-b-sqrt(d))/(2*a);
				Vector3d ray_intersection = ray_origin + t*ray_direction;
				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	write_matrix_to_png(C,C,C,A,filename2);
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-0.5,0.5,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// TODO: Add shading parameter here
				diffuse(i,j) =  (light_position-ray_intersection).normalized().transpose() * ray_normal;
				specular(i,j) = pow(((light_position-ray_intersection).normalized().transpose() * ray_normal),10000);

				// Simple diffuse model
				C(i,j) = ambient + diffuse(i,j) + specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(0.1*C,0.1*C,C,A,filename);
}

int main() {
	// raytrace_sphere();
	// raytrace_parallelogram();
	// raytrace_perspective();
	raytrace_shading();

	return 0;
}

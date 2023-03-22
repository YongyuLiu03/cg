////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
        int self;     // Index of the node itself
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "dragon.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::Node recursive_construct(std::vector<AABBTree::Node> &nodes, const std::vector<int> &fi, const MatrixXd &V, const MatrixXi &F);

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }
   
    // TODO
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    std::vector<int> fi(F.rows());
    std::iota(fi.begin(), fi.end(), 0);
    double span = std::numeric_limits<double>::min();
    int longest_axis = 0;
    for (int i = 0; i < F.cols(); i++) {
        double min = centroids.col(i).minCoeff();
        double max = centroids.col(i).maxCoeff();
        if (span < max - min) {
            span = max - min;
            longest_axis = i;
        }
    }
    std::stable_sort(fi.begin(), fi.end(), [&centroids, &longest_axis](int i1, int i2) {return centroids(i1, longest_axis) < centroids(i2, longest_axis);});
    recursive_construct(nodes, fi, V, F);
    root = nodes.size()-1;         
}

AABBTree::Node recursive_construct(std::vector<AABBTree::Node> &nodes, const std::vector<int> &fi, const MatrixXd &V, const MatrixXi &F) {
    if (fi.size() == 1) {
        AABBTree::Node leaf;
        leaf.bbox = bbox_from_triangle(V.row(F(fi[0], 0)), V.row(F(fi[0], 1)), V.row(F(fi[0], 2)));
        leaf.left = -1;
        leaf.right = -1;
        leaf.triangle = fi[0];
        leaf.self = nodes.size();
        nodes.push_back(leaf);
        return leaf;
    } else {
        const std::size_t half_size = fi.size() / 2;
        const std::vector<int> fi_left(fi.begin(), fi.begin()+half_size);
        const std::vector<int> fi_right(fi.begin()+half_size, fi.end());
        AABBTree::Node T1, T2;
        T1 = recursive_construct(nodes, fi_left, V, F);
        T2 = recursive_construct(nodes, fi_right, V, F);
        AABBTree::Node root;
        root.bbox = T1.bbox.merged(T2.bbox);
        root.left = T1.self;
        root.right = T2.self;
        root.parent = -1;
        root.triangle = -1;
        root.self = nodes.size();
        T1.parent = root.self;
        T2.parent = root.self;
        nodes.push_back(root);
        return root;
    }
}


////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    Vector3d RHS(ray_origin-a);
    Matrix3d LHS;
    LHS << b-a, c-a, -ray_direction;
    Vector3d x = LHS.colPivHouseholderQr().solve(RHS);
    double u, v, t;
    u = x(0);
    v = x(1);
    t = x(2);
    if (u>=0 && v>=0 && u+v<=1 && t>=0) {
        p = ray_origin + t*ray_direction;
        N = (b-a).cross(c-a).normalized();
        return t;
    }
    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    double tx1, tx2, ty1, ty2, tz1, tz2, t_max, t_min;
    tx1 = (box.corner(AlignedBox3d::BottomLeftFloor)[0] - ray_origin(0))/ray_direction(0);
    tx2 = (box.corner(AlignedBox3d::TopRightCeil)[0] - ray_origin(0))/ray_direction(0);
    t_max = std::max(tx1, tx2);
    t_min = std::min(tx1, tx2);

    ty1 = (box.corner(AlignedBox3d::BottomLeftFloor)[1] - ray_origin(1))/ray_direction(1);
    ty2 = (box.corner(AlignedBox3d::TopRightCeil)[1] - ray_origin(1))/ray_direction(1);
    t_max = std::min(t_max, std::max(ty1, ty2));
    t_min = std::max(t_min, std::min(ty1, ty2));

    tz1 = (box.corner(AlignedBox3d::BottomLeftFloor)[2] - ray_origin(2))/ray_direction(2);
    tz2 = (box.corner(AlignedBox3d::TopRightCeil)[2] - ray_origin(2))/ray_direction(2);
    t_max = std::min(t_max, std::max(tz1, tz2));
    t_min = std::max(t_min, std::min(tz1, tz2));
    return (t_max>=t_min && t_min>=0);
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    double closest_t = std::numeric_limits<double>::max();
    int closest_index = -1;
    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // for (int i = 0; i < facets.rows(); i++) {
    //     const Vector3d a = vertices.row(facets(i, 0));
    //     const Vector3d b = vertices.row(facets(i, 1));
    //     const Vector3d c = vertices.row(facets(i, 2)); 
    //     const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
    //     if (t>=0 && t<closest_t) {
    //         closest_t = t;
    //         closest_index = i;
    //         p = tmp_p;
    //         N = tmp_N;
    //     }
    // } 
    // return (closest_index != -1);
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    AABBTree::Node node, left, right;
    std::vector<AABBTree::Node> nodes;
    nodes.push_back(bvh.nodes[bvh.root]);
    std::vector<int> triangles;
    while (nodes.size() != 0) {
        node = nodes.back();
        nodes.pop_back();
        if (node.triangle != -1) {
            triangles.push_back(node.triangle);
        }
        if (node.left != -1){
            left = bvh.nodes[node.left];
            if (ray_box_intersection(ray_origin, ray_direction, left.bbox)) {
                nodes.push_back(left);
            }
        }
        if (node.right != -1){
            right = bvh.nodes[node.right];
            if (ray_box_intersection(ray_origin, ray_direction, right.bbox)) {
                nodes.push_back(right);
            }
        }
    }
    for (int i: triangles) {
        const Vector3d a = vertices.row(facets(i, 0));
        const Vector3d b = vertices.row(facets(i, 1));
        const Vector3d c = vertices.row(facets(i, 2));
        const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
        if (t>=0 && t<closest_t) {
            closest_t = t;
            closest_index = i;
            p = tmp_p;
            N = tmp_N;
        }
    }
    return (closest_index != -1);
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = focal_length * tan(field_of_view/2);
    double image_x = image_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
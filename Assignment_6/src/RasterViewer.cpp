#include "SDLViewer.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <map>
#include <functional>
#include <iostream>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

int main(int argc, char *args[])
{   
    int width = 500;
    int height = 500;
    // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{   
        VertexAttributes out;
        out.position = uniform.view * va.translate * va.origin_translate_back * va.rotate * va.scale * va.origin_translate * va.position;
        out.color = va.color;
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	std::vector<VertexAttributes> vertices;

    char mode;
    std::string edit = "iopc";
    std::string trans = "hjkl";
    std::string view = "=-wasd";
    std::string keyframe = "kabc";

    int i_cnt = 0;
    std::vector<VertexAttributes> i_lines;
    std::vector<VertexAttributes> i_vertices;

    int tri = -1;
    int prev_tri = -1;
    Eigen::Vector4d o_v;
    int is_mouse_pressed = 0;
    std::vector<Eigen::Vector4d> o_prev_color(3);
    double o_trans_x;
    double o_trans_y;

    int c_v = -1;
    std::map<int, Eigen::Vector4d> color_wheel;
    color_wheel[1] << 1, 0, 0, 1;
    color_wheel[2] << 1, (double)102/255, 0, 1;
    color_wheel[3] << 1, 1, 0, 1;
    color_wheel[4] << (double)128/255, 1, 0, 1;
    color_wheel[5] << 0, 1, 0, 1;
    color_wheel[6] << 0, 1, (double)128/255, 1;
    color_wheel[7] << 0, 1, 1, 1;
    color_wheel[8] << 0, (double)102/255, 1, 1;
    color_wheel[9] << (double)128/255, 0, 1, 1;

    double radians = 0.174;
    double scale_ratio = 0.25;
    double view_ratio = 0.2;

    Eigen::Matrix4d view_matrix = Eigen::Matrix4d::Identity();

    std::vector<std::vector<Eigen::Matrix4d>> frame_matrices;

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    viewer.init("Viewer Example", width, height);

    viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
        double vertex_x = (double(x)/double(width)*2)-1;
        double vertex_y = (double(height-1-y)/double(height)*2)-1;
        VertexAttributes v(vertex_x, vertex_y, 0);
        v.position = view_matrix*v.position;
        Eigen::Vector4d trans_pos;
        switch (mode) {
            case 'i':  
                if (i_cnt == 1) {
                    if (i_lines.size() == 2) {
                        i_lines.pop_back();
                    }
                    i_lines.push_back(v);
                } else if (i_cnt == 2) {
                    if (i_lines.size() == 6) {
                        i_lines[3] = v;
                        i_lines[5] = v;
                    } else {
                        i_lines.push_back(i_lines[0]);
                        i_lines.push_back(v);
                        i_lines.push_back(i_lines[1]);
                        i_lines.push_back(v);
                    }
                }
                break;
            case 'o': {
                if (tri == -1) {break;}
                if (is_mouse_pressed) {
                    // trans_pos = (uniform.view*vertices[tri*3].translate*vertices[tri*3].origin_translate_back*vertices[tri*3].rotate*vertices[tri*3].scale*vertices[tri*3].origin_translate).inverse()*v.position;
                    double o_x = (v.position - o_v)[0];
                    double o_y = (v.position - o_v)[1];
                    vertices[tri*3].translate(0, 3) = o_x+o_trans_x;
                    vertices[tri*3+1].translate(0, 3) = o_x+o_trans_x;
                    vertices[tri*3+2].translate(0, 3) = o_x+o_trans_x;
                    vertices[tri*3].translate(1, 3) = o_y+o_trans_y;
                    vertices[tri*3+1].translate(1, 3) = o_y+o_trans_y;
                    vertices[tri*3+2].translate(1, 3) = o_y+o_trans_y;
                }
                break;
            }
            default:
                break;
        }
        viewer.redraw_next = true;
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
        double vertex_x = (double(x)/double(width)*2)-1;
        double vertex_y = (double(height-1-y)/double(height)*2)-1;
        VertexAttributes v(vertex_x, vertex_y, 0);
        v.position = view_matrix*v.position;
        Eigen::Vector4d trans_pos;
        is_mouse_pressed = is_pressed;
        if (is_pressed) {
            prev_tri = tri;
            tri = -1;
            for (int i = 0; i < vertices.size() / 3; i++) {
                trans_pos = (vertices[i*3].translate*vertices[i*3].origin_translate_back*vertices[i*3].rotate*vertices[i*3].scale*vertices[i*3].origin_translate).inverse()*v.position;
                Eigen::Vector2d RHS = (trans_pos-vertices[i*3].position).head(2);
                Eigen::Matrix2d LHS;
                LHS << (vertices[i*3+1].position-vertices[i*3].position).head(2),
                    (vertices[i*3+2].position-vertices[i*3].position).head(2);
                Eigen::Vector2d x = LHS.colPivHouseholderQr().solve(RHS);
                if (x(0) + x(1) <= 1 && x(0) >= 0 && x(1) >= 0) {
                    tri = i;
                    break;
                }
            }
        }
        switch (mode) {
            case 'i':
                if (!is_pressed) {
                    if (i_cnt == 0) {
                        i_lines.push_back(v);
                        i_vertices.push_back(v);
                        i_cnt++;
                    } else if (i_cnt == 1) {
                        i_vertices.push_back(v);
                        i_cnt++;
                    } else if (i_cnt == 2) {
                        i_vertices.push_back(v);
                        Eigen::Vector4d barycenter = (i_vertices[0].position+i_vertices[1].position+i_vertices[2].position)/3;
                        for (auto v: i_vertices) { 
                            v.origin_translate(0, 3) -= barycenter(0);
                            v.origin_translate(1, 3) -= barycenter(1); 
                            v.origin_translate_back(0, 3) += barycenter(0);
                            v.origin_translate_back(1, 3) += barycenter(1);
                            vertices.push_back(v);
                        }
                        i_lines.clear();
                        i_vertices.clear();
                        i_cnt = 0;
                    }
                }
                break;
            case 'o':
                if (is_pressed) {
                    if (prev_tri != -1) {
                        vertices[prev_tri*3].color << o_prev_color[0];
                        vertices[prev_tri*3+1].color << o_prev_color[1];
                        vertices[prev_tri*3+2].color << o_prev_color[2];
                        prev_tri = -1;
                    } 
                    if (tri != -1) {
                        o_v = v.position;
                        o_trans_x = vertices[tri*3].translate(0,3);
                        o_trans_y = vertices[tri*3].translate(1,3);
                        o_prev_color[0] = vertices[tri*3].color;
                        o_prev_color[1] = vertices[tri*3+1].color;
                        o_prev_color[2] = vertices[tri*3+2].color;
                        vertices[tri*3].color << 0,0,1,1;
                        vertices[tri*3+1].color << 0,0,1,1;
                        vertices[tri*3+2].color << 0,0,1,1;
                    }
                }
                break;
            case 'p':
                if (tri == -1) {break;}
                vertices.erase(vertices.begin()+tri*3, vertices.begin()+tri*3+3);
                tri = -1;
                break;
            case 'c': 
                if (is_pressed) {
                    double c_dis = std::numeric_limits<double>::max();
                    for (int i=0; i<vertices.size(); i++) {
                        double dis = (v.position-vertices[i].position).squaredNorm();
                        if (dis <= c_dis) {
                            c_dis = dis;
                            c_v = i;
                        }
                    }
                }
                break;
            default:
                break;
        }
        viewer.redraw_next = true;
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
        if (!is_pressed) {
            if (std::find(edit.begin(), edit.end(), key) != edit.end()) {
                if (key != 'i' && mode == 'i' && i_cnt != 0) {
                    i_lines.clear();
                    i_vertices.clear();
                    i_cnt = 0;
                } else if (key != 'o' && mode == 'o' && tri != -1) {
                    vertices[tri*3].color << o_prev_color[0];
                    vertices[tri*3+1].color << o_prev_color[1];
                    vertices[tri*3+2].color << o_prev_color[2];
                    tri = -1;
                } else if (key != 'c' && mode == 'c' && c_v != -1) {
                    c_v = -1;
                }
                mode = key;
            } else if (std::find(trans.begin(), trans.end(), key) != trans.end()) {
                if (tri == -1) {return;}
                Eigen::Matrix4d rotate = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d scale = Eigen::Matrix4d::Identity();
                switch (key){
                    case 'h': 
                        rotate(0, 0) = std::cos(radians);
                        rotate(1, 1) = std::cos(radians);
                        rotate(0, 1) = std::sin(-radians);
                        rotate(1, 0) = std::sin(radians);
                        break;
                    case 'j':
                        rotate(0, 0) = std::cos(-radians);
                        rotate(1, 1) = std::cos(-radians);
                        rotate(0, 1) = std::sin(radians);
                        rotate(1, 0) = std::sin(-radians);
                        break;
                    case 'k':
                        scale(0, 0) += scale_ratio;
                        scale(1, 1) += scale_ratio;
                        break;
                    case 'l':
                        scale(0, 0) -= scale_ratio;
                        scale(1, 1) -= scale_ratio;
                        break;
                    default:
                        break;
                }
                vertices[tri*3].rotate *= rotate;
                vertices[tri*3+1].rotate *= rotate;
                vertices[tri*3+2].rotate *= rotate;
                vertices[tri*3].scale *= scale;
                vertices[tri*3+1].scale *= scale;
                vertices[tri*3+2].scale *= scale;
                Eigen::Vector4d barycenter = (vertices[tri*3].position+vertices[tri*3+1].position+vertices[tri*3+2].position)/3;
                viewer.redraw_next = true;
            } else if (std::find(view.begin(), view.end(), key) != view.end()) {
                switch (key) {
                    case '=':
                        view_matrix(0, 0) -= view_ratio;
                        view_matrix(1, 1) -= view_ratio;
                        break;
                    case '-':
                        view_matrix(0, 0) += view_ratio;
                        view_matrix(1, 1) += view_ratio;
                        break;
                    case 'w':
                        view_matrix(1, 3) += view_ratio;
                        break;
                    case 'a':
                        view_matrix(0, 3) -= view_ratio;
                        break;
                    case 's':
                        view_matrix(1, 3) -= view_ratio;
                        break;
                    case 'd':
                        view_matrix(0, 3) += view_ratio;
                        break;                    
                    default:
                        break;
                }
                uniform.view = view_matrix.inverse();
                std::cout << "view_matrix\n"<<view_matrix<<std::endl;
                viewer.redraw_next = true;
            } else if ((key-'0') >= 1 && (key-'0') <= 9 && c_v != -1) {
                vertices[c_v].color = color_wheel[key-'0'];
                std::cout << "c_v:" << c_v << " key:" << (key-'0') << " vertices[c_v].color\n" << vertices[c_v].color << std::endl;
                viewer.redraw_next = true;
            } else if ((modifier & KMOD_SHIFT) && std::find(keyframe.begin(), keyframe.end(), key) != keyframe.end()) {
                switch (key) {
                    case 'k':
                        frame_matrices.push_back(std::vector<Eigen::Matrix4d>(vertices.size()));
                        for (int i=0; i<vertices.size(); i++) {
                            Eigen::Matrix4d transform_matrix = vertices[i].translate * vertices[i].origin_translate_back * vertices[i].rotate * vertices[i].scale * vertices[i].origin_translate;
                            frame_matrices[-1][i] = transform_matrix;
                        }
                        break;
                    case 'c':
                        
                        break;
                    case 'a':
                        break;
                    case 'b':
                        break;
                    default:
                        break;
                }
            }
        }
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        // Clear the framebuffer
        for (unsigned i=0;i<frameBuffer.rows();i++)
            for (unsigned j=0;j<frameBuffer.cols();j++)
                frameBuffer(i,j).color << 0,0,0,1;

       	rasterize_triangles(program,uniform,vertices,frameBuffer);
        rasterize_lines(program,uniform,i_lines,1,frameBuffer);
        // Buffer for exchanging data between rasterizer and sdl viewer
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

        for (unsigned i=0; i<frameBuffer.rows();i++)
        {
            for (unsigned j=0; j<frameBuffer.cols();j++)
            {
                R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
                G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
                B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
                A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
            }
        }
        viewer.draw_image(R, G, B, A);
    };

    viewer.launch();

    return 0;
}

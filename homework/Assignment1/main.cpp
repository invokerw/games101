#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float theta = rotation_angle/180*M_PI;
    Eigen::Matrix4f rotate;
    rotate <<    std::cos(theta), -std::sin(theta), 0, 0, 
                    std::sin(theta), std::cos(theta), 0, 0, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1;

    model = rotate * model;

    return model;
}


Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float theta = angle/180*M_PI;;

    // Rodrigues' Rotation Formula
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N;  // n 的反对称矩阵
    N <<         0, -axis.z(),  axis.y(),
          axis.z(),         0, -axis.x(),
         -axis.y(),  axis.x(),         0;

    Eigen::Matrix3f rotation_matrix =   cos(theta)*I + 
                                        (1-cos(theta))*axis*axis.transpose() + 
                                        sin(theta)*N;

    model.block<3,3>(0,0) = rotation_matrix;  // block size of (3,3), starting as (0,0)

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float half_eye_angel_radian = eye_fov/2.0/180.0*M_PI;
    float t = zNear*std::tan(half_eye_angel_radian); //top 
    float r = t*aspect_ratio; //right 
    float l = -r; //left 
    float b = -t; //bottom 
    float n = zNear;
    float f = zFar;

    Eigen::Matrix4f persp_ortho;
    persp_ortho <<  n, 0, 0, 0, 
                    0, n, 0, 0, 
                    0, 0, n+f, -n*f, 
                    0, 0, 1, 0;

    

    Eigen::Matrix4f ortho1;
    ortho1  <<  2/(r-l), 0, 0, 0,
                0, 2/(t-b), 0, 0,
                0, 0, 2/(n-f), 0,
                0, 0, 0, 1;
    Eigen::Matrix4f ortho2;
    ortho2 <<   1, 0, 0, -1*(r+l)/2,
                0, 1, 0, -1*(t+b)/2,
                0, 0, 1, -1*(n+f)/2,
                0,0,0,1;    
    Eigen::Matrix4f ortho = ortho1 * ortho2;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    projection = ortho * persp_ortho;    
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    //Eigen::Vector3f axis = {1, 2, 3};

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        //r.set_model(get_rotation(axis, angle));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}

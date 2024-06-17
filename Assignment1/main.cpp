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
    Eigen::Matrix4f R;
    float A = rotation_angle * MY_PI / 180;
    R << cos(A), -sin(A), 0, 0,
        sin(A), cos(A), 0, 0,
        0, 0, 1, 0,
        0, 0, 0,1;
    model = R * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    std::cout << eye_fov <<' ' << aspect_ratio << zNear << zFar;
    float F =  1 / tan(180 / MY_PI * eye_fov  / 2 / 180.0 * MY_PI );
    //构建正交矩阵
    Eigen::Matrix4f T;
    T << 1 / (aspect_ratio * zNear * tan(eye_fov / 2)), 0, 0, 0, 
        0, 1 / (zNear * tan(eye_fov / 2)), 0, 0, 
        0, 0, 2 / (zNear - zFar), 0, 
        0, 0, 0, 1;

    //构建透视矩阵
    Eigen::Matrix4f P;
    P << zNear, 0, 0, 0, 
        0, zNear, 0, 0, 
        0, 0, zNear + zFar, zNear* zFar, 
        0, 0, 1, 0;

    projection = P * projection;
    return projection;
}

Eigen::Matrix3f get_rotation(Vector3f axis,float angle) 
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    axis.normalize();
    float rad = angle * MY_PI / 180.0; 
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);
    std::cout << axis << ' ' << angle << std::endl;;
    Eigen::Matrix3f RA;
    RA << axis.x()* axis.x()*(1 - cosA)+ cosA, axis.x()* axis.y()* (1 - cosA)-axis.z()*sinA, axis.x()* axis.z()* (1 - cosA) + axis.y() * sinA,
        axis.y()*axis.x()*(1-cosA)+axis.z()*sinA, axis.y()* axis.y()* (1 - cosA) + cosA, axis.y()* axis.z()* (1 - cosA) - axis.z() * sinA,
        axis.z()* axis.x()* (1 - cosA) - axis.y() * sinA, axis.z()* axis.y()* (1 - cosA) + axis.x() * sinA, axis.z()* axis.z()* (1 - cosA) + cosA;
    rotation = RA * rotation;
    std::cout << rotation << std::endl;;
    return rotation;
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
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    //实现罗德里格斯旋转公式
    Eigen::Vector3f D(0,1,0);
    Eigen::Matrix3f rotationMatrix = get_rotation(D, 45);
    Eigen::Vector3f vector(1, 0, 0); 
    Eigen::Vector3f rotatedVector = rotationMatrix * vector;
    std::cout << "Rotated Vector: \n" << rotatedVector << std::endl;
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

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
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

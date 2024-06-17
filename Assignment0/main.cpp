#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

//int main(){
//
//    // Basic Example of cpp
//   /* std::cout << "Example of cpp \n";
//    float a = 1.0, b = 2.0;
//    std::cout << a << std::endl;
//    std::cout << a/b << std::endl;
//    std::cout << std::sqrt(b) << std::endl;
//    std::cout << std::acos(-1) << std::endl;
//    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;*/
//
//    // Example of vector
//    std::cout << "Example of vector \n";
//    // vector definition
//    Eigen::Vector3f v(1.0f,2.0f,3.0f);
//    Eigen::Vector3f w(1.0f,0.0f,0.0f);
//    // vector output
//    std::cout << "Example of output \n";
//    std::cout << v << std::endl;
//    // vector add
//    std::cout << "Example of add \n";
//    std::cout << v + w << std::endl;
//    // vector scalar multiply
//    std::cout << "Example of scalar multiply \n";
//    std::cout << v * 3.0f << std::endl;
//    std::cout << 2.0f * v << std::endl;
//
//    // Example of matrix
//    std::cout << "Example of matrix \n";
//    // matrix definition
//    Eigen::Matrix3f i,j;
//    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
//    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
//    // matrix output
//    std::cout << "Example of output \n";
//    std::cout << i << std::endl;
//    // matrix add i + j
//    // matrix scalar multiply i * 2.0
//    // matrix multiply i * j
//    // matrix multiply vector i * v
//
//    return 0;
//}

int main() {
	Eigen::Vector3f p(2.0f, 1.0f, 1.0f);
	float R = 45 * 3.14 / 180;
	Eigen::Matrix3f P1;
	P1 << cos(R), -sin(R), 0, sin(R), cos(R), 0, 0, 0, 1;
	P1 * p;
	Eigen::Matrix3f P2;
	P2 << 1, 0, 1, 0, 1, 2, 0, 0, 1;
	std::cout <<P2 * P1 * p;
	return 0;
}


//Eigen::Vector3f RotateByZ(Eigen::Vector3f pos, float rotation)
//{
//    float a = rotation * 3.14 / 180;
//    Eigen::Matrix3f rot;
//    rot << cos(a), -sin(a), 0.0,
//        sin(a), cos(a), 0.0,
//        0.0, 0.0, 1.0;
//    return rot * pos;
//}
//
//Eigen::Vector3f TranslateXY(Eigen::Vector3f pos, float x, float y)
//{
//    Eigen::Matrix3f trans;
//    trans << 1, 0, x,
//        0, 1, y,
//        0, 0, 1;
//    return trans * pos;
//}
//
//int main()
//{
//    Eigen::Vector3f pos(2.0f, 1.0f, 1.0f);
//    std::cout << TranslateXY(RotateByZ(pos, 45), 1, 2) << std::endl;
//}
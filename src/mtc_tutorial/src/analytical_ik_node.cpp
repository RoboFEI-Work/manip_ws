#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "mtc_tutorial/analytical_ik.hpp"

using Mat4 = std::array<std::array<double, 4>, 4>;

Mat4 identity()
{
    return {{{1.0, 0.0, 0.0, 0.0},
             {0.0, 1.0, 0.0, 0.0},
             {0.0, 0.0, 1.0, 0.0},
             {0.0, 0.0, 0.0, 1.0}}};
}

Mat4 multiply(const Mat4 & a, const Mat4 & b)
{
    Mat4 out = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k)
            {
                sum += a[i][k] * b[k][j];
            }
            out[i][j] = sum;
        }
    }
    return out;
}

Mat4 rotZ(double t)
{
    const double c = std::cos(t);
    const double s = std::sin(t);
    return {{{c, -s, 0.0, 0.0},
             {s, c, 0.0, 0.0},
             {0.0, 0.0, 1.0, 0.0},
             {0.0, 0.0, 0.0, 1.0}}};
}

Mat4 rotY(double t)
{
    const double c = std::cos(t);
    const double s = std::sin(t);
    return {{{c, 0.0, s, 0.0},
             {0.0, 1.0, 0.0, 0.0},
             {-s, 0.0, c, 0.0},
             {0.0, 0.0, 0.0, 1.0}}};
}

Mat4 trans(double x, double y, double z)
{
    Mat4 t = identity();
    t[0][3] = x;
    t[1][3] = y;
    t[2][3] = z;
    return t;
}

void printMat(const char * name, const Mat4 & m)
{
    std::cout << name << " =\n";
    for (int i = 0; i < 4; ++i)
    {
        std::cout << "  [ ";
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::fixed << std::setprecision(6) << std::setw(10) << m[i][j];
            if (j < 3)
            {
                std::cout << " ";
            }
        }
        std::cout << " ]\n";
    }
    std::cout << '\n';
}

class AnalyticalIkNode : public rclcpp::Node
{
public:
    AnalyticalIkNode()
    : Node("analytical_ik_node")
    {
        // Parametros de alvo para IK generica em XYZ.
        declare_parameter<double>("x", 0.30);
        declare_parameter<double>("y", 0.00);
        declare_parameter<double>("z", 0.25);
        declare_parameter<bool>("elbow_up", false);

        const double x = get_parameter("x").as_double();
        const double y = get_parameter("y").as_double();
        const double z = get_parameter("z").as_double();
        const bool elbow_up = get_parameter("elbow_up").as_bool();

        const auto solution = manip_ik::solveIK(
            x,
            y,
            z,
            elbow_up);

        if (!solution)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Ponto fora do workspace para x=%.3f y=%.3f z=%.3f elbow_up=%s",
                x,
                y,
                z,
                elbow_up ? "true" : "false");
            return;
        }

        RCLCPP_INFO(get_logger(), "D = %.6f", solution->D);
        RCLCPP_INFO(get_logger(), "j1 = %.6f", solution->joint1);
        RCLCPP_INFO(get_logger(), "j2 = %.6f", solution->joint2);
        RCLCPP_INFO(get_logger(), "j3 = %.6f", solution->joint3);
        RCLCPP_INFO(get_logger(), "j4 = %.6f", solution->joint4);
        RCLCPP_INFO(get_logger(), "j5 = %.6f", solution->joint5);
        RCLCPP_INFO(get_logger(), "target xyz = (%.3f, %.3f, %.3f)", x, y, z);
        RCLCPP_INFO(get_logger(), "TCP orientation fixed: pitch=pi/2 yaw=0");

        // Cadeia extraida do URDF:
        // T01 = Rz(j1)
        // T12 = Tz(0.10) * Ry(j2)
        // T23 = Tz(0.20) * Ry(j3)
        // T34 = Tz(0.20) * Ry(j4)
        // T45 = Tz(0.10) * Rz(j5)
        // T5tcp = Tz(0.15)
        const Mat4 T01 = rotZ(solution->joint1);
        const Mat4 T12 = multiply(trans(0.0, 0.0, 0.10), rotY(solution->joint2));
        const Mat4 T23 = multiply(trans(0.0, 0.0, 0.20), rotY(solution->joint3));
        const Mat4 T34 = multiply(trans(0.0, 0.0, 0.20), rotY(solution->joint4));
        const Mat4 T45 = multiply(trans(0.0, 0.0, 0.10), rotZ(solution->joint5));
        const Mat4 T5tcp = trans(0.0, 0.0, 0.15);

        const Mat4 T02 = multiply(T01, T12);
        const Mat4 T03 = multiply(T02, T23);
        const Mat4 T04 = multiply(T03, T34);
        const Mat4 T05 = multiply(T04, T45);
        const Mat4 T0tcp = multiply(T05, T5tcp);

        printMat("T01", T01);
        printMat("T12", T12);
        printMat("T23", T23);
        printMat("T34", T34);
        printMat("T45", T45);
        printMat("T5tcp", T5tcp);
        printMat("T0tcp", T0tcp);

        std::cout << "TCP xyz = ("
                  << T0tcp[0][3] << ", "
                  << T0tcp[1][3] << ", "
                  << T0tcp[2][3] << ")\n";
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnalyticalIkNode>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
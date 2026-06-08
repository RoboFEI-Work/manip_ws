#pragma once

#include <cmath>
#include <optional>

namespace manip_ik
{

struct IkSolution
{
    double D;
    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double joint5;
};

inline std::optional<IkSolution> solveIK(
    double x,
    double y,
    double z,
    double tool_pitch = M_PI_2,
    double tool_yaw = 0.0,
    bool elbow_up = false)
{
    // =========================
    // GEOMETRIA DO ROBÔ
    // =========================

    constexpr double BASE_Z = 0.10;

    constexpr double L1 = 0.20;
    constexpr double L2 = 0.20;
    constexpr double L3 = 0.25;

    // =========================
    // JOINT 1 (BASE)
    // =========================

    double joint1 = std::atan2(y, x);

    // =========================
    // DISTÂNCIA RADIAL (PROJECAO NO PLANO XY)
    // =========================

    double r = std::sqrt(x * x + y * y);

    // =========================
    // WRIST CENTER
    // Usa a orientacao desejada da ferramenta (pitch/yaw)
    // =========================

    double rw = r - L3 * std::cos(tool_pitch);

    double zw = (z - BASE_Z) - L3 * std::sin(tool_pitch);

    // =========================
    // LEI DOS COSSENOS
    // =========================

    double D =
        (rw * rw + zw * zw -
         L1 * L1 - L2 * L2) /
        (2.0 * L1 * L2);

    // Fora do workspace
    if (D < -1.0 || D > 1.0)
        return std::nullopt;

    // =========================
    // JOINT 3 (ELBOW)
    // =========================

    double s3 =
        std::sqrt(1.0 - D * D);

    if (elbow_up)
        s3 = -s3;

    double joint3 =
        std::atan2(s3, D);

    // =========================
    // JOINT 2 (SHOULDER)
    // =========================

    double joint2 =
        std::atan2(zw, rw)
        -
        std::atan2(
            L2 * std::sin(joint3),
            L1 + L2 * std::cos(joint3));

    // =========================
    // JOINT 4 (WRIST PITCH)
    // Fecha orientacao de pitch do efetuador
    // =========================

    double joint4 =
        tool_pitch
        - joint2
        - joint3;

    // =========================
    // JOINT 5 (WRIST YAW)
    // =========================

    double joint5 = tool_yaw;

    // =========================
    // LIMITES DO URDF
    // =========================

    if (joint2 < -1.8 || joint2 > 1.8)
        return std::nullopt;

    if (joint3 < -2.6 || joint3 > 2.6)
        return std::nullopt;

    if (joint4 < -2.2 || joint4 > 2.2)
        return std::nullopt;

    return IkSolution{
        D,
        joint1,
        joint2,
        joint3,
        joint4,
        joint5
    };
}

inline std::optional<IkSolution> solveIK(
    double x,
    double y,
    double z,
    bool elbow_up)
{
    // IK generica por posicao (XYZ), mantendo orientacao padrao do TCP.
    return solveIK(x, y, z, M_PI_2, 0.0, elbow_up);
}

} // namespace manip_ik
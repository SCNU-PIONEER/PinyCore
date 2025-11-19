#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "Kinematic.hpp"
#include <cstdio>

using namespace Catch::Matchers;
using namespace ARM;

Matrix<3, 3> extractRotation(const Matrix<4, 4> &transform)
{
    Matrix<3, 3> rotation;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotation(i, j) = transform(i, j);
        }
    }
    return rotation;
}

bool isRotationMatrixValid(const Matrix<3, 3> &rotation)
{
    // 检查行列式是否接近±1（允许反射）
    float det = rotation(0, 0) * (rotation(1, 1) * rotation(2, 2) - rotation(1, 2) * rotation(2, 1)) -
                rotation(0, 1) * (rotation(1, 0) * rotation(2, 2) - rotation(1, 2) * rotation(2, 0)) +
                rotation(0, 2) * (rotation(1, 0) * rotation(2, 1) - rotation(1, 1) * rotation(2, 0));

    return std::abs(std::abs(det) - 1.0f) < 1e-6f;
}

// 验证齐次变换矩阵有效性
bool isTransformValid(const Matrix<4, 4> &transform)
{
    // 检查最后一行
    if (std::abs(transform(3, 0)) > 1e-6f)
        return false;
    if (std::abs(transform(3, 1)) > 1e-6f)
        return false;
    if (std::abs(transform(3, 2)) > 1e-6f)
        return false;
    if (std::abs(transform(3, 3) - 1.0f) > 1e-6f)
        return false;

    // 检查旋转部分
    auto rotation = extractRotation(transform);
    return isRotationMatrixValid(rotation);
}

TEST_CASE("rotation2euler test", "[rotation2euler]")
{
    Kinematic kinematic;
    array<float, 9> identity = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    auto euler = kinematic.rotation2euler(identity);

    SECTION("单位矩阵应该返回零欧拉角")
    {
        // 单位旋转矩阵
        std::array<float, 9> identity_rotation = {
            1.0f, 0.0f, 0.0f, // 第一行
            0.0f, 1.0f, 0.0f, // 第二行
            0.0f, 0.0f, 1.0f  // 第三行
        };

        auto euler = kinematic.rotation2euler(identity_rotation);

        REQUIRE_THAT(euler[0], WithinAbs(0.0f, 1e-6f)); // Roll
        REQUIRE_THAT(euler[1], WithinAbs(0.0f, 1e-6f)); // Pitch
        REQUIRE_THAT(euler[2], WithinAbs(0.0f, 1e-6f)); // Yaw
    }

    SECTION("绕X轴旋转90度")
    {
        std::array<float, 9> rot_x_90 = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f };

        auto euler = kinematic.rotation2euler(rot_x_90);

        REQUIRE_THAT(euler[0], WithinAbs(M_PI_2, 1e-6f)); // Roll = 90°
        REQUIRE_THAT(euler[1], WithinAbs(0.0f, 1e-6f));   // Pitch = 0°
        REQUIRE_THAT(euler[2], WithinAbs(0.0f, 1e-6f));   // Yaw = 0°
    }

    SECTION("绕Y轴旋转45度")
    {
        float cos45 = cosf(M_PI / 4);
        float sin45 = sinf(M_PI / 4);

        std::array<float, 9> rot_y_45 = { cos45, 0.0f, sin45, 0.0f, 1.0f, 0.0f, -sin45, 0.0f, cos45 };

        auto euler = kinematic.rotation2euler(rot_y_45);

        REQUIRE_THAT(euler[0], WithinAbs(0.0f, 1e-6f));     // Roll
        REQUIRE_THAT(euler[1], WithinAbs(M_PI / 4, 1e-6f)); // Pitch = 45°
        REQUIRE_THAT(euler[2], WithinAbs(0.0f, 1e-6f));     // Yaw
    }

    SECTION("绕Z轴旋转30度")
    {
        float cos30 = cosf(M_PI / 6);
        float sin30 = sinf(M_PI / 6);

        std::array<float, 9> rot_z_30 = { cos30, -sin30, 0.0f, sin30, cos30, 0.0f, 0.0f, 0.0f, 1.0f };

        auto euler = kinematic.rotation2euler(rot_z_30);

        REQUIRE_THAT(euler[0], WithinAbs(0.0f, 1e-6f));     // Roll
        REQUIRE_THAT(euler[1], WithinAbs(0.0f, 1e-6f));     // Pitch
        REQUIRE_THAT(euler[2], WithinAbs(M_PI / 6, 1e-6f)); // Yaw = 30°
    }

    SECTION("万向节锁测试 - Pitch = +90°")
    {
        std::array<float, 9> gimbal_lock_pos = { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f };

        auto euler = kinematic.rotation2euler(gimbal_lock_pos);

        REQUIRE_THAT(euler[0], WithinAbs(0.0f, 1e-6f));   // Roll = 0
        REQUIRE_THAT(euler[1], WithinAbs(M_PI_2, 1e-6f)); // Pitch = 90°
        REQUIRE(euler[2] >= -M_PI);
        REQUIRE(euler[2] <= M_PI);
    }

    SECTION("万向节锁测试 - Pitch = -90°")
    {
        std::array<float, 9> gimbal_lock_neg = { 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f };

        auto euler = kinematic.rotation2euler(gimbal_lock_neg);

        REQUIRE_THAT(euler[0], WithinAbs(0.0f, 1e-6f));    // Roll = 0
        REQUIRE_THAT(euler[1], WithinAbs(-M_PI_2, 1e-6f)); // Pitch = -90°
        REQUIRE(euler[2] >= -M_PI);
        REQUIRE(euler[2] <= M_PI);
    }

    SECTION("边界条件测试 - 接近万向节锁")
    {
        float epsilon = 0.0001f;
        std::array<float, 9> test_cases[] = { { epsilon, 0.0f, 1.0f - epsilon, 0.0f, 1.0f, 0.0f, -0.9999f, 0.0f,
                                                epsilon },
                                              { 0.001f, 0.0f, 0.999f, 0.0f, 1.0f, 0.0f, -0.999f, 0.0f, 0.001f },
                                              { 1e-5f, 0.0f, 0.99999f, 0.0f, 1.0f, 0.0f, -0.99999f, 0.0f, 1e-5f } };

        for (const auto &rot_matrix : test_cases) {
            auto euler = kinematic.rotation2euler(rot_matrix);

            const float tolerance = 1e-5f;

            REQUIRE(euler[1] >= -M_PI_2 - tolerance);
            REQUIRE(euler[1] <= M_PI_2 + tolerance);

            REQUIRE(std::isfinite(euler[0]));
            REQUIRE(std::isfinite(euler[1]));
            REQUIRE(std::isfinite(euler[2]));
        }
    }

    SECTION("万向节锁精确测试")
    {
        std::array<float, 9> exact_gimbal_lock = { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f };

        auto euler = kinematic.rotation2euler(exact_gimbal_lock);

        REQUIRE_THAT(euler[0], WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(euler[1], WithinAbs(M_PI_2, 1e-6f));
        REQUIRE(euler[2] >= -M_PI);
        REQUIRE(euler[2] <= M_PI);
    }


    SECTION("欧拉角转旋转矩阵再转回欧拉角 - 一致性测试")
    {
        std::array<std::array<float, 3>, 6> test_euler_angles = {
            std::array<float, 3>{ 0.0f, 0.0f, 0.0f },              // 零角度
            std::array<float, 3>{ M_PI / 6, 0.0f, 0.0f },          // Roll = 30°
            std::array<float, 3>{ 0.0f, M_PI / 4, 0.0f },          // Pitch = 45°
            std::array<float, 3>{ 0.0f, 0.0f, M_PI / 3 },          // Yaw = 60°
            std::array<float, 3>{ M_PI / 4, M_PI / 6, M_PI / 8 },  // 复合旋转
            std::array<float, 3>{ -M_PI / 3, -M_PI / 4, M_PI / 2 } // 负角度
        };

        const float tolerance = 1e-5f;

        for (const auto &original_euler : test_euler_angles) {
            INFO("测试欧拉角: Roll=" << original_euler[0] << ", Pitch=" << original_euler[1]
                                     << ", Yaw=" << original_euler[2]);

            // 欧拉角 → 旋转矩阵
            auto rotation_matrix = kinematic.eulerAngle2Rotation(original_euler);

            // 旋转矩阵 → 欧拉角
            auto recovered_euler = kinematic.rotation2euler(rotation_matrix);

            // 验证一致性
            REQUIRE_THAT(recovered_euler[0], WithinAbs(original_euler[0], tolerance));
            REQUIRE_THAT(recovered_euler[1], WithinAbs(original_euler[1], tolerance));
            REQUIRE_THAT(recovered_euler[2], WithinAbs(original_euler[2], tolerance));
        }
    }
}
TEST_CASE("dh test", "[dh]")
{
    SECTION("单位变换测试")
    {
        // a=0, alpha=0, d=0, theta=0 -> 单位矩阵
        auto transform = Kinematic::dhTransform(0.0f, 0.0f, 0.0f, 0.0f);

        REQUIRE_THAT(transform(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(1, 1), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(2, 2), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(3, 3), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(0, 3), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("纯平移测试")
    {
        // a=0.5, alpha=0, d=0.2, theta=0 -> 纯平移
        auto transform = Kinematic::dhTransform(0.5f, 0.0f, 0.2f, 0.0f);

        REQUIRE_THAT(transform(0, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(1, 1), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(2, 2), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(0, 3), WithinAbs(0.5f, 1e-6f)); // x方向平移
        REQUIRE_THAT(transform(2, 3), WithinAbs(0.2f, 1e-6f)); // z方向平移
    }

    SECTION("纯旋转测试")
    {
        // a=0, alpha=0, d=0, theta=π/2 -> 绕z轴旋转90°
        auto transform = Kinematic::dhTransform(0.0f, 0.0f, 0.0f, M_PI / 2);

        REQUIRE_THAT(transform(0, 0), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(transform(0, 1), WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(transform(1, 0), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(1, 1), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("绕x轴旋转测试")
    {
        // a=0, alpha=π/2, d=0, theta=0 -> 绕x轴旋转90°
        auto transform = Kinematic::dhTransform(0.0f, M_PI / 2, 0.0f, 0.0f);

        REQUIRE_THAT(transform(1, 1), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(transform(1, 2), WithinAbs(-1.0f, 1e-6f));
        REQUIRE_THAT(transform(2, 1), WithinAbs(1.0f, 1e-6f));
        REQUIRE_THAT(transform(2, 2), WithinAbs(0.0f, 1e-6f));
    }

    SECTION("数学正确性验证")
    {
        auto transform = Kinematic::dhTransform(0.3f, M_PI / 4, 0.2f, M_PI / 6);

        // 验证变换矩阵有效性
        REQUIRE(isTransformValid(transform));

        // 验证最后一行是[0,0,0,1]
        REQUIRE_THAT(transform(3, 0), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(transform(3, 1), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(transform(3, 2), WithinAbs(0.0f, 1e-6f));
        REQUIRE_THAT(transform(3, 3), WithinAbs(1.0f, 1e-6f));
    }

    SECTION("边界条件测试")
    {
        SECTION("零参数测试")
        {
            auto transform = Kinematic::dhTransform(0.0f, 0.0f, 0.0f, 0.0f);
            REQUIRE(isTransformValid(transform));
        }

        SECTION("极端角度测试")
        {
            auto transform = Kinematic::dhTransform(0.1f, M_PI, 0.2f, -M_PI);
            REQUIRE(isTransformValid(transform));
        }
    }
}

TEST_CASE("fk test", "[fk]")
{
    SECTION("基本功能验证")
    {
        // 使用MDH参数
        std::array<float, 6> theta = { 0.0f, M_PI_2, -M_PI_2, 0.0f, -M_PI_2, 0.0f };
        std::array<float, 6> a = { 0.0f, 0.0f, 5.f, 5.f, 0.0f, 0.0f };
        std::array<float, 6> alpha = { 0.0f, M_PI_2, 0.0f, 0, -M_PI_2, -M_PI_2 };
        std::array<float, 6> d = { 0.0f, 0.0f, 0.0f, 1.f, 1.f, 1.f };

        MDH mdh_params(theta, a, alpha, d);
        Kinematic kinematic(mdh_params);

        SECTION("零位姿态基本验证")
        {
            std::array<float, 6> zero_angles = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            auto transform = kinematic.fkine2T(zero_angles);

            REQUIRE(isTransformValid(transform));

            REQUIRE_THAT(transform(3, 0), WithinAbs(0.0f, 1e-6f));
            REQUIRE_THAT(transform(3, 1), WithinAbs(0.0f, 1e-6f));
            REQUIRE_THAT(transform(3, 2), WithinAbs(0.0f, 1e-6f));
            REQUIRE_THAT(transform(3, 3), WithinAbs(1.0f, 1e-6f));

            REQUIRE(std::isfinite(transform(0, 3)));
            REQUIRE(std::isfinite(transform(1, 3)));
            REQUIRE(std::isfinite(transform(2, 3)));
        }

        SECTION("可重复性测试")
        {
            std::array<float, 6> test_angles = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f };

            auto result1 = kinematic.fkine2T(test_angles);
            auto result2 = kinematic.fkine2T(test_angles);
            auto result3 = kinematic.fkine2T(test_angles);

            REQUIRE(result1 == result2);
            REQUIRE(result2 == result3);
            REQUIRE(result1 == result3);
        }

        SECTION("正运动学测试")
        {
            std::array<float, 6> test_angles = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f };

            auto T = kinematic.fkine2T(test_angles);

            REQUIRE_THAT(T(0, 3), WithinAbs(3.192953f, 1e-6f));  // x
            REQUIRE_THAT(T(1, 3), WithinAbs(-0.202824f, 1e-6f)); // y
            REQUIRE_THAT(T(2, 3), WithinAbs(8.606504f, 1e-6f));  // z
        }
    }
}

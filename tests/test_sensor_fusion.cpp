#include <gtest/gtest.h>
#include "emulator/peripherals/sensor_fusion.hpp"
#include "emulator/utils/types.hpp"
#include <memory>
#include <cmath>
#include <vector>

namespace emulator::peripherals::test {

class SensorFusionTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto fusion_result = SensorFusion::create();
        ASSERT_TRUE(fusion_result.has_value()) << "Failed to create sensor fusion system";
        sensor_fusion = std::move(fusion_result.value());
        
        ASSERT_TRUE(sensor_fusion->initialize().has_value()) 
            << "Failed to initialize sensor fusion system";
    }
    
    void TearDown() override {
        if (sensor_fusion) {
            sensor_fusion->shutdown();
        }
    }
    
    std::unique_ptr<SensorFusion> sensor_fusion;
    
    SensorFusion::IMUData create_test_imu_data(float ax, float ay, float az, 
                                               float gx, float gy, float gz) {
        SensorFusion::IMUData data;
        data.accelerometer = {ax, ay, az};
        data.gyroscope = {gx, gy, gz};
        data.timestamp = std::chrono::steady_clock::now();
        return data;
    }
    
    SensorFusion::MagnetometerData create_test_mag_data(float mx, float my, float mz) {
        SensorFusion::MagnetometerData data;
        data.magnetic_field = {mx, my, mz};
        data.timestamp = std::chrono::steady_clock::now();
        return data;
    }
};

TEST_F(SensorFusionTest, BasicInitialization) {
    auto config = sensor_fusion->get_configuration();
    ASSERT_TRUE(config.has_value());
    
    EXPECT_GT(config.value().sample_rate, 0);
    EXPECT_EQ(config.value().algorithm, SensorFusion::Algorithm::MADGWICK);
}

TEST_F(SensorFusionTest, ConfigurationUpdate) {
    SensorFusion::Configuration new_config;
    new_config.sample_rate = 200;
    new_config.algorithm = SensorFusion::Algorithm::KALMAN;
    new_config.beta = 0.1f;
    
    EXPECT_TRUE(sensor_fusion->configure(new_config).has_value());
    
    auto updated_config = sensor_fusion->get_configuration();
    ASSERT_TRUE(updated_config.has_value());
    EXPECT_EQ(updated_config.value().sample_rate, 200);
    EXPECT_EQ(updated_config.value().algorithm, SensorFusion::Algorithm::KALMAN);
    EXPECT_FLOAT_EQ(updated_config.value().beta, 0.1f);
}

TEST_F(SensorFusionTest, MadgwickFilterBasic) {
    SensorFusion::Configuration config;
    config.algorithm = SensorFusion::Algorithm::MADGWICK;
    config.beta = 0.1f;
    EXPECT_TRUE(sensor_fusion->configure(config).has_value());
    
    auto imu_data = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    
    auto orientation = sensor_fusion->get_orientation();
    ASSERT_TRUE(orientation.has_value());
    
    EXPECT_NEAR(orientation.value().w, 1.0f, 0.1f);
    EXPECT_NEAR(orientation.value().x, 0.0f, 0.1f);
    EXPECT_NEAR(orientation.value().y, 0.0f, 0.1f);
    EXPECT_NEAR(orientation.value().z, 0.0f, 0.1f);
}

TEST_F(SensorFusionTest, KalmanFilterBasic) {
    SensorFusion::Configuration config;
    config.algorithm = SensorFusion::Algorithm::KALMAN;
    EXPECT_TRUE(sensor_fusion->configure(config).has_value());
    
    for (int i = 0; i < 10; ++i) {
        auto imu_data = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f);
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    }
    
    auto orientation = sensor_fusion->get_orientation();
    ASSERT_TRUE(orientation.has_value());
    
    float magnitude = std::sqrt(orientation.value().w * orientation.value().w +
                               orientation.value().x * orientation.value().x +
                               orientation.value().y * orientation.value().y +
                               orientation.value().z * orientation.value().z);
    EXPECT_NEAR(magnitude, 1.0f, 0.01f);
}

TEST_F(SensorFusionTest, ComplementaryFilter) {
    SensorFusion::Configuration config;
    config.algorithm = SensorFusion::Algorithm::COMPLEMENTARY;
    config.alpha = 0.98f;
    EXPECT_TRUE(sensor_fusion->configure(config).has_value());
    
    for (int i = 0; i < 20; ++i) {
        auto imu_data = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.1f, 0.0f, 0.0f);
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    }
    
    auto angles = sensor_fusion->get_euler_angles();
    ASSERT_TRUE(angles.has_value());
    
    EXPECT_GT(std::abs(angles.value().roll), 0.01f);
}

TEST_F(SensorFusionTest, MagnetometerIntegration) {
    auto imu_data = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f);
    auto mag_data = create_test_mag_data(0.2f, 0.0f, -0.4f);
    
    EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    EXPECT_TRUE(sensor_fusion->update_magnetometer(mag_data).has_value());
    
    auto orientation = sensor_fusion->get_orientation();
    ASSERT_TRUE(orientation.has_value());
    
    auto heading = sensor_fusion->get_heading();
    ASSERT_TRUE(heading.has_value());
    EXPECT_GE(heading.value(), 0.0f);
    EXPECT_LT(heading.value(), 360.0f);
}

TEST_F(SensorFusionTest, OrientationTracking) {
    std::vector<SensorFusion::Quaternion> orientations;
    
    for (int i = 0; i < 100; ++i) {
        float t = i * 0.01f;
        float gx = 0.5f * std::sin(t);
        float gy = 0.3f * std::cos(t);
        
        auto imu_data = create_test_imu_data(0.0f, 0.0f, 9.81f, gx, gy, 0.0f);
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
        
        auto orientation = sensor_fusion->get_orientation();
        ASSERT_TRUE(orientation.has_value());
        orientations.push_back(orientation.value());
    }
    
    EXPECT_EQ(orientations.size(), 100);
    
    for (const auto& q : orientations) {
        float magnitude = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        EXPECT_NEAR(magnitude, 1.0f, 0.01f);
    }
}

TEST_F(SensorFusionTest, EulerAngleConversion) {
    auto imu_data = create_test_imu_data(9.81f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    for (int i = 0; i < 50; ++i) {
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    }
    
    auto euler = sensor_fusion->get_euler_angles();
    ASSERT_TRUE(euler.has_value());
    
    EXPECT_NEAR(euler.value().roll, 90.0f, 5.0f);
    EXPECT_NEAR(euler.value().pitch, 0.0f, 5.0f);
    EXPECT_NEAR(euler.value().yaw, 0.0f, 5.0f);
}

TEST_F(SensorFusionTest, AngularVelocityIntegration) {
    float angular_velocity = 1.0f;
    
    for (int i = 0; i < 100; ++i) {
        auto imu_data = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, angular_velocity);
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    }
    
    auto euler_final = sensor_fusion->get_euler_angles();
    ASSERT_TRUE(euler_final.has_value());
    
    EXPECT_GT(std::abs(euler_final.value().yaw), 10.0f);
}

TEST_F(SensorFusionTest, CalibrationSystem) {
    EXPECT_TRUE(sensor_fusion->start_calibration().has_value());
    
    for (int i = 0; i < 200; ++i) {
        float ax = (i % 20 - 10) * 0.1f;
        float ay = (i % 30 - 15) * 0.05f;
        float az = 9.81f + (i % 10 - 5) * 0.02f;
        
        auto imu_data = create_test_imu_data(ax, ay, az, 0.0f, 0.0f, 0.0f);
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    }
    
    EXPECT_TRUE(sensor_fusion->complete_calibration().has_value());
    
    auto calibration = sensor_fusion->get_calibration_data();
    ASSERT_TRUE(calibration.has_value());
    
    EXPECT_NEAR(calibration.value().accel_bias.x, 0.0f, 0.1f);
    EXPECT_NEAR(calibration.value().accel_bias.y, 0.0f, 0.1f);
}

TEST_F(SensorFusionTest, MotionDetection) {
    auto imu_data_stationary = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f);
    
    for (int i = 0; i < 50; ++i) {
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data_stationary).has_value());
    }
    
    auto motion_state = sensor_fusion->get_motion_state();
    ASSERT_TRUE(motion_state.has_value());
    EXPECT_EQ(motion_state.value(), SensorFusion::MotionState::STATIONARY);
    
    auto imu_data_moving = create_test_imu_data(2.0f, 1.0f, 9.81f, 0.5f, 0.3f, 0.2f);
    
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data_moving).has_value());
    }
    
    motion_state = sensor_fusion->get_motion_state();
    ASSERT_TRUE(motion_state.has_value());
    EXPECT_EQ(motion_state.value(), SensorFusion::MotionState::MOVING);
}

TEST_F(SensorFusionTest, FreeFallDetection) {
    auto imu_data_normal = create_test_imu_data(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f);
    
    for (int i = 0; i < 20; ++i) {
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data_normal).has_value());
    }
    
    auto motion_state = sensor_fusion->get_motion_state();
    ASSERT_TRUE(motion_state.has_value());
    EXPECT_NE(motion_state.value(), SensorFusion::MotionState::FREE_FALL);
    
    auto imu_data_freefall = create_test_imu_data(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    for (int i = 0; i < 30; ++i) {
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data_freefall).has_value());
    }
    
    motion_state = sensor_fusion->get_motion_state();
    ASSERT_TRUE(motion_state.has_value());
    EXPECT_EQ(motion_state.value(), SensorFusion::MotionState::FREE_FALL);
}

TEST_F(SensorFusionTest, PerformanceMetrics) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 1000; ++i) {
        auto imu_data = create_test_imu_data(
            0.1f * std::sin(i * 0.01f),
            0.1f * std::cos(i * 0.01f),
            9.81f,
            0.01f * std::sin(i * 0.02f),
            0.01f * std::cos(i * 0.02f),
            0.01f * std::sin(i * 0.03f)
        );
        
        EXPECT_TRUE(sensor_fusion->update_imu(imu_data).has_value());
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    double updates_per_second = 1000000.0 / duration.count() * 1000.0;
    EXPECT_GT(updates_per_second, 10000.0);
    
    auto stats = sensor_fusion->get_performance_stats();
    ASSERT_TRUE(stats.has_value());
    EXPECT_EQ(stats.value().total_updates, 1000);
    EXPECT_GT(stats.value().average_update_time, 0);
}

TEST_F(SensorFusionTest, ErrorHandling) {
    SensorFusion::IMUData invalid_data;
    invalid_data.accelerometer = {std::numeric_limits<float>::infinity(), 0.0f, 0.0f};
    invalid_data.gyroscope = {0.0f, 0.0f, 0.0f};
    invalid_data.timestamp = std::chrono::steady_clock::now();
    
    auto result = sensor_fusion->update_imu(invalid_data);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), utils::ErrorCode::INVALID_DATA);
}

} // namespace emulator::peripherals::test
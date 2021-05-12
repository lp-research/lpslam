#pragma once

#include "DataTypes/Space.h"
#include "Trackers/TrackerBase.h"
#include "Utils/ManagedThread.h"
#include "DataTypes/CameraQueue.h"
#include "DataTypes/SensorQueue.h"
#include "Sources/ImageSourceBase.h"
#include "Utils/PidController.h"

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Field.hpp>

#include <string>

#define BUMPERS_NUMBER 2
#define CLIFF_SENSORS_NUMBER 4


namespace LpSlam {

/**
 * Webot data source working together with the e-pucks IMU
 * Coordinate system of the webot is the following when looking
 * down on the robot:
 * 
 * +y forward direction (driving direction)
 * +z point up
 * +x pointing to the right of the robot
 */
class WebotsSource : public ImageSourceBase{
public:

    WebotsSource();

    bool startSensor(SensorQueue &) override;

    bool start(CameraQueue &) override;

    void stop() override;

private:
    enum class RoboState {
        Driving,
        Backwards,
        TurnLeft,
        TurnRight,
        DriveToWaypoint,
        Stop
    };

    struct WorkerThreadParams {
        CameraQueue * camQ;
        SensorQueue * sensorQ;
        webots::Supervisor * robo;

        webots::Accelerometer * accelerometer;
        webots::Gyro * gyro;
        webots::Camera * cam;
        webots::Camera * cam_second = nullptr;

        webots::Motor * left_motor;
        webots::Motor * right_motor;

        webots::PositionSensor * left_position_sensor;
        webots::PositionSensor * right_position_sensor;

        webots::TouchSensor * bumpers[BUMPERS_NUMBER];
        webots::DistanceSensor * cliff_sensors[CLIFF_SENSORS_NUMBER];

        webots::Node * robo_node;
        webots::Node * camera_node;
        webots::Field * trans_field;
        webots::Field * rot_field;

        double speed[2];
        double sensors_value[8];
        double braitenberg_coefficients[8][2] = {{0.942, -0.22}, {0.63, -0.1}, {0.5, -0.06},  {-0.06, -0.06},
                                                {-0.06, -0.06}, {-0.06, 0.5}, {-0.19, 0.63}, {-0.13, 0.942}};
        webots::DistanceSensor * distance_sensor[8];

        TimeStamp initialTime;

        RoboState state = RoboState::Driving;
        float waitFor = 0.0f;
        // 2 ms timesteps
        // WorldInfo.basicTimeStep in Webots must be same or smaller than this number
        int timestep = 4;
        int * total_time = nullptr;

        std::vector<Position3> waypoints;
        PidController * pid;
    };

    int m_total_time = 0;
    SensorQueue * m_sensorQ = nullptr;

    const std::string m_configWaypoints = "waypoints";

    ManagedThread<WorkerThreadParams> m_worker;
    std::unique_ptr<webots::Supervisor> m_robo;
    PidController m_pid;
};

}
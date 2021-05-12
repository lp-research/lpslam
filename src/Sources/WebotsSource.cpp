#include "WebotsSource.h"

#include "Utils/TimeMeasurement.h"

#include <spdlog/spdlog.h>
#include <cstdlib>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <iostream>
#include <random>
#include <cmath>

using namespace LpSlam;

/** roomba controller
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
*/

#define BUMPER_LEFT 0
#define BUMPER_RIGHT 1
static const char *bumpers_name[BUMPERS_NUMBER] = {"bumper_left", "bumper_right"};

#define CLIFF_SENSOR_LEFT 0
#define CLIFF_SENSOR_FRONT_LEFT 1
#define CLIFF_SENSOR_FRONT_RIGHT 2
#define CLIFF_SENSOR_RIGHT 3
static const char *cliff_sensors_name[CLIFF_SENSORS_NUMBER] = {"cliff_left", "cliff_front_left", "cliff_front_right",
                                                               "cliff_right"};

#define MAX_SPEED 16.0
#define NULL_SPEED 0
#define HALF_SPEED 8
#define MIN_SPEED -16

#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

namespace NavigationHelper {

    inline double computeDistanceTarget( Position3 roboPos, Position3 target ){
        return (target.value - roboPos.value).norm();
    }

    inline double computeDirectionAngle( Position3 roboPos, Orientation roboOrientation, Position3 target ){

        //std::cout << "roboPos " << roboPos.value << std::endl;
        //std::cout << "target " << target.value << std::endl;
        //std::cout << "roboOrientation " << roboOrientation.value.x() << std::endl;

        // compute relative distance between robo and target
        const Vector3 targetDirection = (target.value - roboPos.value).normalized();

        //std::cout << "targetDirection " << targetDirection << std::endl;

        // construct direction vector of robot
        const Vector3 roboDirection = roboOrientation.value.inverse() * Vector3(0.0, 0.0, 1.0);

        //std::cout << "roboDirection " << roboDirection << std::endl;

        // compute angle between vectors, unsigned
        double angle = std::acos(targetDirection.dot(roboDirection));

        // compute direction of angle
        const Vector3 cross = targetDirection.cross(roboDirection);

        if (cross.x() < 0) {
            angle = - angle;
        }
        //std::cout << "angle " << angle << std::endl;
        return angle;
    }
}

WebotsSource::WebotsSource() : 
    m_worker( [](WebotsSource::WorkerThreadParams & params) -> bool {


        std::random_device rd;
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.0, 0.8);
        std::uniform_real_distribution<> dis_rot(0.0, 0.2);

        std::normal_distribution<> dis_pos_mod(0.0, 0.02);
        std::normal_distribution<> dis_quat_mod(0.0, 0.01);
        std::normal_distribution<> dis_vel_mod(0.0, 0.02);

        bool applySmearing = false;

        double speed[2];
        double sensors_value[8];
        double braitenberg_coefficients[8][2] = {{0.942, -0.22}, {0.63, -0.1}, {0.5, -0.06},  {-0.06, -0.06},
                                                {-0.06, -0.06}, {-0.06, 0.5}, {-0.19, 0.63}, {-0.13, 0.942}};

        const double cRange = (1024 / 2);
        
        if (params.robo->step(params.timestep) == -1) {
            return false;
        }
        *params.total_time += params.timestep;

        const double *a = params.accelerometer->getValues();

        // unit is rad/s
        const double *g = params.gyro->getValues();

        // dump out sensor values
        SPDLOG_DEBUG("Got Imu measurement from Webots: {}, {}, {}", g[0],
            g[1], g[2]);

        auto thisTimestampMs = std::chrono::milliseconds( *params.total_time );
        auto absoluteTimestamp = params.initialTime + thisTimestampMs;

        TrackerCoordinateSystemBase base;
        SensorQueueEntry qentry(absoluteTimestamp,
            base,
            Acceleration3(a[0], a[1], a[2]),
            AngularVelocity3(g[0], g[1], g[2]));
        params.sensorQ->push(qentry);

        // get all the statistics from the camera node
        // because that is where we run the position reconstruction
        const double * veloArray = params.camera_node->getVelocity();
        const double * posArray = params.camera_node->getPosition();
        const double * orientArray = params.camera_node->getOrientation();

        Vector3 velo_w( veloArray[0], veloArray[1], veloArray[2]);
        //std::cout << "Robot Velocity (unrot)" << velo_w << std::endl;

        /*
        Webots coordinates system is:
        +z : forward
        +x : left
        +y : up
        in the robot direction if the robot is rotated by 180 degrees
        around the y axis. In the robots unrotated coordinate frame
        it would drive backwards because forward is in the -z direction
        */

        Vector3 pos_w( posArray[1], -posArray[0], posArray[2]);
        //std::cout << "Robot Position (unrot)" << pos_w << std::endl;

        Eigen::Matrix3d Rwc;
        // transforms a point from the robo frame in the world frame
        Rwc << orientArray[0], orientArray[1], orientArray[2],
            orientArray[3], orientArray[4], orientArray[5],
            orientArray[6], orientArray[7], orientArray[8];
        //std::cout << "Robot Rwc" << Rwc << std::endl;

        // push robot position on queue
        GlobalState gs_sim;
        // switch orientation, we need orientation world frame -> robot frame
        Quaternion quat_webots_frame(Rwc.transpose());
        // rotate y around 180 degrees because the iRobots forward direction is in the wrong direction
        Quaternion quat_rot_to_forward;
        quat_rot_to_forward = Eigen::AngleAxisd(3.1415, Eigen::Vector3d::UnitY());
        quat_webots_frame = quat_rot_to_forward * quat_webots_frame;

        Quaternion quat_lpgf_frame(quat_webots_frame.w(),
            quat_webots_frame.y(), -quat_webots_frame.x(), quat_webots_frame.z());

        //std::cout << "Robot Output quaternion " << quat_lpgf_frame.w() << " " << quat_lpgf_frame.x() << " "
        //    << quat_lpgf_frame.y() << " "<< quat_lpgf_frame.z() << " "<< std::endl;
        //std::cout << "Robot Output Matrix " << quat_lpgf_frame.toRotationMatrix() << std::endl;
        gs_sim.orientation = LpSlam::Orientation(quat_lpgf_frame);
        gs_sim.position = LpSlam::Position3(pos_w);
        // velocity is in relation to the robot orientation
        gs_sim.velocity = LpSlam::Vector3(quat_lpgf_frame * velo_w);
        gs_sim.velocityValid = true;

        if (applySmearing) {
            gs_sim.position.value = gs_sim.position.value +
                LpSlam::Vector3(dis_pos_mod(gen),
                    dis_pos_mod(gen),
                    dis_pos_mod(gen));
            gs_sim.orientation.value = Quaternion(
                gs_sim.orientation.value.w() + dis_quat_mod(gen),
                gs_sim.orientation.value.x() + dis_quat_mod(gen),
                gs_sim.orientation.value.y() + dis_quat_mod(gen),
                gs_sim.orientation.value.z() + dis_quat_mod(gen)
            );
            gs_sim.orientation.value.normalize();
            gs_sim.velocity.value = gs_sim.velocity.value +
            LpSlam::Vector3(dis_vel_mod(gen),
                    dis_vel_mod(gen),
                    dis_vel_mod(gen));
        }

        SensorQueueEntry qentry_gs(absoluteTimestamp,
            base,
            gs_sim);
        params.sensorQ->push(qentry_gs);

        // get camera image
        // we do 10 Hz with the camera
        // total time is in milliseconds
        // TODO: for some reason I don't understand, the Webots simulation gets very slow
        // as soon as the camera image get read out. But its not the reading out the camera
        // image call but every simulation step gets very slow ...
        // Also seems to be not related to the camera readout rate
        if (((*params.total_time) % 50) == 0) {
            TimingBase t_meas_all_image("all_webot_image");
            //std::cout << "Storing image at time " << (*params.total_time) << std::endl;
            auto w = params.cam->getWidth();
            auto h = params.cam->getHeight();
            // image is BGRA format
            auto byteSizeImage = w * h * 4;
            //std::cout << "Image has size " << w << " x " << h << std::endl;
            const auto camSize = cv::Size(w, h);

            TimingBase t_meas_load("loading_webot_image");
            t_meas_load.report();

             {
                TimingBase t_meas("convert_webot_image");

                auto lmdGetImage = [camSize, &params](webots::Camera * cam) {
                    const unsigned char * imageData = cam->getImage();
                    cv::Mat camImageBGRA(camSize, CV_8UC4, (void*)imageData);
                    cv::Mat camGray;
                    cv::cvtColor(camImageBGRA, camGray, cv::COLOR_BGRA2GRAY);
                    return camGray;
                };

                auto camImageFirst = lmdGetImage(params.cam);
                //std::cout << "Convert Image of size " << w << " x " << h << std::endl;
                // convert to 
                //cv::Mat camImage(320, 240, CV_8UC3, cv::Scalar(0, 0, 0));

                // do we have a second camera ?
                CameraQueueEntry camEntry;

                if (params.cam_second) {
                    auto camImageSecond = lmdGetImage(params.cam_second);

                    TrackerCoordinateSystemBase base;
                    camEntry = CameraQueueEntry(absoluteTimestamp,
                        0,
                        base,
                        camImageFirst,
                        1,
                        base,
                        camImageSecond
                        );

                } else {
                    TrackerCoordinateSystemBase base;
                    camEntry = CameraQueueEntry(absoluteTimestamp,
                        base,
                        camImageFirst
                        );
                }

                camEntry.globalState = gs_sim;

                params.camQ->push(camEntry);
                //std::cout << "Image pushed on queue" << std::endl;
                t_meas.report();
            }
            t_meas_all_image.report();
        }

    if (params.waitFor > 0.0f) {
      params.waitFor -= float(params.timestep) * 0.001f;
      return true;
    }

    // no movement any more
    if (params.state == RoboState::Stop) {
        return true;
    }

    if (params.state == RoboState::TurnRight) {
      params.left_motor->setVelocity(-HALF_SPEED);
      params.right_motor->setVelocity(HALF_SPEED);
      params.waitFor = 0.94 + dis_rot(gen);

        if (params.waypoints.size() == 0) {
            params.state = RoboState::Driving;
        } else {
            params.state = RoboState::DriveToWaypoint;
        }
      return true;
    }

    if (params.state == RoboState::TurnLeft) {
      params.left_motor->setVelocity(HALF_SPEED);
      params.right_motor->setVelocity(-HALF_SPEED);
      params.waitFor = 0.94 + dis_rot(gen);
        if (params.waypoints.size() == 0) {
            params.state = RoboState::Driving;
        } else {
            params.state = RoboState::DriveToWaypoint;
        }
      return true;
    }

    if (params.bumpers[BUMPER_LEFT]->getValue() != 0.0) {
        params.left_motor->setVelocity(-HALF_SPEED);
        params.right_motor->setVelocity(-HALF_SPEED);
      params.waitFor = 0.2 + dis(gen);
      params.state = RoboState::TurnRight;
      return true;
    } else if (params.bumpers[BUMPER_RIGHT]->getValue() != 0.0) {
        params.left_motor->setVelocity(-HALF_SPEED);
        params.right_motor->setVelocity(-HALF_SPEED);
      params.waitFor = 0.2 + dis(gen);
      params.state = RoboState::TurnLeft;
      return true;
    }

    // regular driving
    if (params.state == RoboState::DriveToWaypoint) {
        // check if we are at waypoint
        if (params.waypoints.size() == 0) {
            params.state = RoboState::Stop;

            params.left_motor->setVelocity(0.0);
            params.right_motor->setVelocity(0.0);

            return true;
        }

        auto thisWapyoint = params.waypoints.front();
        auto dist = NavigationHelper::computeDistanceTarget(gs_sim.position, thisWapyoint);
        std::cout << "dis " << dist << std::endl;
        if ( dist < 0.15) {
            // next waypoint !
            params.waypoints.erase(params.waypoints.begin());
            std::cout << "moving next waypoint: " << params.waypoints.front().value << std::endl;
            return true;
        }

        // compute driving direction to this waypoint
        auto direction = NavigationHelper::computeDirectionAngle(gs_sim.position, gs_sim.orientation,
            thisWapyoint);

        double minSpeed = 0.5 * MAX_SPEED;
        //double directionRatio = std::abs(direction) / boost::math::constants::pi<double>();

        double left_mod = 1.0;
        double right_mod = 1.0;

        double control = - params.pid->computeControl(0.0, direction);
        std::cout << " direction : " << direction << " control : " << control << std::endl;
        std::cout << " Robot position " << gs_sim.position.value << std::endl;
        std::cout << " Driving to waypoint " << thisWapyoint.value << std::endl;
        // only steer if we are not on target yet.
        /*if (std::abs(direction) > 0.05)*/ //{
/*            std::cout << " std::sin(direction) " << std::sin(direction) << std::endl;
            if (direction > 0) {
                right_mod = std::max(control, 0.4);
            } else {
                left_mod = std::max(control, 0.4);
            }
        }*/
        double controlRatioLeft = 1.0 - (control / 3.14);
        double controlRatioRight = 1.0 + (control / 3.14);

        std::cout << "mods " << left_mod << " " << right_mod << std::endl;
        // motor are switched
        params.left_motor->setVelocity(std::clamp( MAX_SPEED * controlRatioLeft, 0.4, MAX_SPEED));
        params.right_motor->setVelocity(std::clamp( MAX_SPEED * controlRatioRight, 0.4, MAX_SPEED));

    } else if (params.state == RoboState::Driving) {
        params.left_motor->setVelocity(MAX_SPEED);
        params.right_motor->setVelocity(MAX_SPEED);
    }

        return true; 
    } ), m_pid(1.5, 0.2, 0.2) {

        getConfigOptions().optional(m_configWaypoints, std::vector<Position3>());
}

bool WebotsSource::startSensor(SensorQueue & q) {
    m_sensorQ = &q;
    return true;
}

bool WebotsSource::start(CameraQueue & q) {
    WorkerThreadParams threadParams;

    threadParams.waypoints = getConfigOptions().getPositionList(m_configWaypoints);
    std::cout << "Loaded " << threadParams.waypoints.size() << " waypoints" << std::endl;

    std::cout << "creating robot class" << std::endl;
    m_robo = std::make_unique<webots::Supervisor>();
    std::cout << " robot class created " << std::endl;

    if (threadParams.waypoints.size() > 0) {
        threadParams.state = RoboState::DriveToWaypoint;
    }

    threadParams.pid = &m_pid;
    threadParams.robo = m_robo.get();

    // its important to get the Robots's main object (which is of type Robot)
    // which is a solid type and not to use the m_robo->getSelf()
    // The DEF name ROBOT1 needs to be set for the Robot in the Webots GUI
    // also make sure that the supervisor flag is on for the robot
    std::cout << "ROBOT1 will be loaded" << std::endl;
    auto rootBot = m_robo->getFromDef("ROBOT1");
    // for this function to work we need at least Webots 2020B
    auto cameraSolid = m_robo->getFromProtoDef("ROBOT1.CREATE_CAMERA");

    if (rootBot == nullptr) {
        spdlog::error("Cannot load ROBOT1");
        return false;
    }

    if (cameraSolid == nullptr) {
        spdlog::error("Cannot load CREATE_CAMERA");
        return false;
    }

    std::cout << "ROBOT1 loaded" << std::endl;

    threadParams.robo_node = rootBot;
    threadParams.camera_node = cameraSolid;
    threadParams.trans_field = rootBot->getField("translation");
    threadParams.rot_field = rootBot->getField("rotation");

    if (m_sensorQ == nullptr) {
        spdlog::error("Sensore queue not set in Websot source");
        return false;
    }
    threadParams.camQ = &q;
    threadParams.sensorQ = m_sensorQ;
    m_total_time = 0;
    threadParams.total_time = &m_total_time;
    threadParams.initialTime = std::chrono::high_resolution_clock::now();

    spdlog::info("Starting to enable sensors and actuators on robot");
    threadParams.cam = m_robo->getCamera("camera");
    // cam has 20 Hz, we only read out the frame every 50 ms
    threadParams.cam->enable(50);
    std::cout << "CAM name " << threadParams.cam->getName() << std::endl;

    // check for second camera
    threadParams.cam_second = m_robo->getCamera("camera_second");
    if (threadParams.cam_second) {
        threadParams.cam_second->enable(50);
    }

    threadParams.accelerometer = m_robo->getAccelerometer("accelerometer");
    threadParams.accelerometer->enable(threadParams.timestep);
    threadParams.gyro = m_robo->getGyro("gyro");
    threadParams.gyro->enable(threadParams.timestep);
 
    for (int i = 0; i < BUMPERS_NUMBER; i++) {
        threadParams.bumpers[i] = m_robo->getTouchSensor(bumpers_name[i]);
        threadParams.bumpers[i]->enable(threadParams.timestep);
    }

    for (int i = 0; i < CLIFF_SENSORS_NUMBER; i++) {
        threadParams.cliff_sensors[i] = m_robo->getDistanceSensor(cliff_sensors_name[i]);
        threadParams.cliff_sensors[i]->enable(threadParams.timestep);
    }

    threadParams.left_motor = m_robo->getMotor("left wheel motor");
    threadParams.right_motor = m_robo->getMotor("right wheel motor");
    threadParams.left_motor->setPosition(INFINITY);
    threadParams.right_motor->setPosition(INFINITY);
    threadParams.left_motor->setVelocity(0.0);
    threadParams.right_motor->setVelocity(0.0);
  
    threadParams.left_position_sensor = m_robo->getPositionSensor("left wheel sensor");
    threadParams.right_position_sensor = m_robo->getPositionSensor("right wheel sensor");
    threadParams.left_position_sensor->enable(threadParams.timestep);
    threadParams.right_position_sensor->enable(threadParams.timestep);

    spdlog::info("Robot activation complete");
/*
    for (int i = 0; i < 8; i++) {
        char device_name[4];

        sprintf(device_name, "ps%d", i);
        threadParams.distance_sensor[i] = m_robo->getDistanceSensor(device_name);
        threadParams.distance_sensor[i]->enable(threadParams.timestep);
    }
*/
    m_worker.start(threadParams);
    spdlog::info("Frame aquisition with Webots started");
    return true;
}

void WebotsSource::stop() {
    m_worker.stop();

    spdlog::info("Frame aquisition with Webots stopped");
}
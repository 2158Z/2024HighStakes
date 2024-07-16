#include "main.h"
#include "vars.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMG({0,0,0}, pros::MotorGearset::red);
pros::MotorGroup rightMG({0,0,0}, pros::MotorGearset::red);

pros::Rotation horizontalSensor(0); // Horizontal Sensor
pros::Rotation verticalSensor(0); // Vertical Sensor

lemlib::Drivetrain drivetrain(
	&leftMG,
 	&rightMG,
	0, // Track width
	lemlib::Omniwheel::NEW_325,	// Wheel type
	0, // RPM
	2 // Horizontal Drift
);

pros::IMU imu(0);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
	0, // integral gain (kI)
	3, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	3, // large error range, in inches
	500, // large error range timeout, in milliseconds
	20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
	0, // integral gain (kI)
	10, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

lemlib::TrackingWheel horizontalTrackingWheel( // Horizontal Tracking Wheel
	&horizontalSensor, 
	lemlib::Omniwheel::NEW_275, 
	0 // Offset
	);

lemlib::TrackingWheel verticalTrackingWheel( // Vertical Tracking Wheel
	&verticalSensor, 
	lemlib::Omniwheel::NEW_275, 
	0 // Offset
	);

lemlib::OdomSensors sensors(
	&verticalTrackingWheel, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	&horizontalTrackingWheel, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
	&imu // inertial sensor
	);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(
	3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(
	3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors, // odometry sensors
	&throttleCurve,
	&steerCurve
	);

void initialize() {
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
}

void opcontrol() {
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // chassis.curvature(leftY, rightX);
        pros::delay(25);
    }
}
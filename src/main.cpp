#include "main.h"
#include "pros/motors.h"
#include "screen.h"
#include "util.h"
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMG({20,19,10}, pros::MotorGearset::blue);
pros::MotorGroup rightMG({-11,-12,-1}, pros::MotorGearset::blue);

pros::Motor intake(0, pros::MotorGears::blue);

pros::Rotation horizontalSensor(0); // Horizontal Sensor
pros::Rotation verticalSensor(0); // Vertical Sensor

lemlib::Drivetrain drivetrain(
	&leftMG,
 	&rightMG,
	30, // Track width
	lemlib::Omniwheel::NEW_325,	// Wheel type
	1200, // RPM
	2 // Horizontal Drift
);

pros::IMU imu(0);

lemlib::Pose pose(0,0,0);

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

lemlib::TrackingWheel verticalTrackingWheel( // Vertical Tracking Wheel
	&verticalSensor, 
	lemlib::Omniwheel::NEW_275, 
	0 // Offset
	);

lemlib::OdomSensors sensors(
	&verticalTrackingWheel, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	nullptr, // horizontal tracking wheel 1
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
	sensors // odometry sensors
	// &throttleCurve,
	// &steerCurve
);

void initialize() {
	screen::main();
	leftMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void global_variable(){
	pose = chassis.getPose();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
}

std::vector<float> arcadeControl(double leftInput, double rightInput) {
	// output voltages of left and right in vector
	std::vector<float> voltages = {0, 0};

	rightInput = util::clamp(rightInput, -0.5, 0.5);

	float max = std::max(fabs(leftInput), fabs(rightInput));
	float difference = leftInput - rightInput;
	float total = leftInput + rightInput;

	if(leftInput >= 0) {
		if(rightInput >= 0) {
			voltages = std::vector<float> {max, difference};
		} else {
			voltages = std::vector<float> {total, max};
		}
	} else {
		if(rightInput >= 0) {
			voltages = std::vector<float> {total, -max};
		} else {
			voltages = std::vector<float> {-max, difference};
		}
	}

	return voltages;
}

float easeInOutExpo(float x) {
	return x == 0 // If x is 0
	? 0
	: x == 1 // If x is 1
	? 1
	: x < 0.5 ? pow(2, 20 * x - 10) / 2
	: (2 - pow(2, -20 * x + 10)) / 2;
}

void opcontrol() {
	intake.move_voltage(0);

    while (true) {
        // get left y and right x positions
        float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)/127;
        float rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/127;

        // move the robot
        chassis.arcade(easeInOutExpo(leftY) * util::sgn(leftY) * 127, easeInOutExpo(rightX) * util::sgn(rightX) * 127, false, 0.75);
	
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move_voltage(12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move_voltage(-12000);
		}
		pros::delay(25);
	}
}
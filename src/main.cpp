#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/motors.h"
#include "misc/screen.h"
#include "util.h"
#include "PID.h"
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMG({-6,-8,-10}, pros::MotorGearset::blue);
pros::MotorGroup rightMG({2,3,4}, pros::MotorGearset::blue);

pros::Motor intake(16, pros::MotorGears::blue);
pros::Motor conveyor(1, pros::MotorGearset::blue);

lemlib::Pose pose = lemlib::Pose(0,0);

pros::adi::DigitalOut clamp('D');
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut climbUp('B');
pros::adi::DigitalOut climbDown('A');

int conveyorToggle = false;
bool clampToggle = false;
int conveyorDirection = 1;
bool hangUp = false;
bool hangDown = false;
bool doinkerToggle = false;

pros::Rotation verticalSensor(15); // Vertical Sensor

lemlib::Drivetrain drivetrain(
	&leftMG,
 	&rightMG,
	12.5, // Track width
	lemlib::Omniwheel::NEW_275,	// Wheel type
	450, // RPM
	2 // Horizontal Drift
);

std::vector<float> driveConstants = {6000, 0.17, 0.0005, 1, 2, 75, 0.25, 1000}; //1.25
std::vector<float> turnConstants = {12000, 0.015, 0.00, 0.103, 2, 75, 0.75, 1000}; //.0075

pros::IMU imu(19);

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
	0 // maximum acceleration (slew)
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

lemlib::TrackingWheel verticalTrackingWheel( 
	&verticalSensor,
	lemlib::Omniwheel::NEW_275,
	0 // Offset
	);

lemlib::OdomSensors sensors(
	&verticalTrackingWheel, // vertical tracking wheel 1, set to null
	nullptr,
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

void driveVoltage(float leftVoltage, float rightVoltage){
	leftMG.move_voltage(leftVoltage);
	rightMG.move_voltage(rightVoltage);
}

void driveDistance(float distance, float timeout, std::vector<float> dConstants = driveConstants) {
	PID leftPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);
	PID rightPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);

	leftMG.tare_position();
	rightMG.tare_position();
	int counter = 0;
	// float lastRightOutput = 0;
	// float lastLeftOutput = 0;
	// float lastRightAddition = 0;
	// float lastLeftAddition = 0;

	while(!leftPID.is_settled() || !rightPID.is_settled()) {
		float leftTraveled = leftMG.get_position() / 360 * M_PI * 3.25 * .75; 
		float rightTraveled = rightMG.get_position() / 360 * M_PI * 3.25 * .75;
		
		float rightError = distance - rightTraveled;
		float leftError = distance - leftTraveled;
		

		float leftOutput = leftPID.compute(leftError) * 10000;
		float rightOutput = rightPID.compute(rightError) * 10000;

		leftOutput = util::clamp(leftOutput, -driveConstants[0], driveConstants[0]);
		rightOutput = util::clamp(rightOutput, -driveConstants[0], driveConstants[0]);

		// if(rightOutput > (lastRightOutput + lastRightAddition)) {
		//     rightOutput = lastRightOutput + lastRightAddition;
		//     lastRightAddition += 100;
		// }
		// if(leftOutput > lastLeftOutput + lastLeftAddition) {
		//     leftOutput = lastLeftOutput + lastLeftAddition;
		//     lastLeftAddition += 100;
		// }

		driveVoltage(leftOutput, rightOutput); 

		// lastRightOutput = rightOutput;
		// lastLeftOutput = leftOutput;

		printf("%f %f %f %f \n", leftError, rightError, leftOutput, rightOutput);
		counter++;
		delay(10);
	}
	driveVoltage(0,0);
	printf("%s", "settled");
}

void turnAngle(float angle, std::vector<float> tConstants = turnConstants) {        // relative
	PID turnPID(angle, tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);
	float relativeHeading = 0;
	float absHeading = imu.get_heading();
	float previousAbsHeading = absHeading;
	while (!turnPID.is_settled()) {
		float deltaAngle = 0;
		absHeading = imu.get_heading();

		deltaAngle = absHeading - previousAbsHeading;
		if(deltaAngle < -180 || deltaAngle > 180) { //if it crosses from 0 to 360 or vice versa
			deltaAngle = -360 + absHeading + previousAbsHeading;
		}

		relativeHeading += deltaAngle;
		float error = angle - relativeHeading;
		previousAbsHeading = absHeading;

		float output = turnPID.compute(error) * 10000;


		output = util::clamp(output, -tConstants[0], tConstants[0]);
		driveVoltage(output, -output);
		printf("%f %f\n", error, output);
		delay(10);
	}
	printf("%s", "settled");
}

void initialize() {
	LVGL_screen::main();
	chassis.calibrate();
	leftMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void disabled() {}

void competition_initialize() {}
void autonomous() {
	switch(LVGL_screen::autonID * LVGL_screen::side){
		case 1: // Blue Preload
			pros::delay(200);
			leftMG.move_voltage(-4000);
			rightMG.move_voltage(-4000);
			pros::delay(1500);
			leftMG.move_voltage(0);
			rightMG.move_voltage(0);
			// driveDistance(-16, 5000);
			
			clamp.set_value(!clampToggle);
			pros::delay(250);
			conveyor.move_voltage(-10000);
			turnAngle(67.5);  // Left side 112.5, -67.5 Right side
			intake.move_voltage(-12000);
			driveDistance(36, 1000);
			pros::delay(2000);
			break;
		case 2: // Blue Corner
			chassis.setPose(60,-12,90);
			chassis.moveToPose(20, -24, 60, 2000, {.forwards = false, .minSpeed = 10});
			clamp.set_value(!clampToggle);
			conveyor.move_voltage(-8000);
			intake.move_voltage(12000);
			chassis.turnToPoint(24, -52, 2000);
			chassis.moveToPoint(24,-52, 2000);
			chassis.moveToPoint(60,-60, 2000);
			intake.move_voltage(0);
			break;
		case -1: // Red Preload
			pros::delay(200);
			leftMG.move_voltage(-4000);
			rightMG.move_voltage(-4000);
			pros::delay(1500);
			leftMG.move_voltage(0);
			rightMG.move_voltage(0);
			// driveDistance(-16, 5000);
			
			clamp.set_value(!clampToggle);
			pros::delay(250);
			conveyor.move_voltage(-10000);
			turnAngle(-67.5);  // Left side 112.5, -67.5 Right side
			intake.move_voltage(-12000);
			driveDistance(36, 1000);
			pros::delay(2000);
			// turnAngle(-110);
			// intake.move_voltage(12000);
			// driveDistance(30, 500);
			break;
		case -2: // Red corner
			chassis.setPose(-60,-12,270);
			chassis.moveToPose(-20, -24, 300, 2000, {.forwards = false, .minSpeed = 10});
			clamp.set_value(!clampToggle);
			conveyor.move_voltage(-8000);
			intake.move_voltage(12000);
			chassis.turnToPoint(-24, -52, 2000);
			chassis.moveToPoint(-24,-52, 2000);
			chassis.moveToPoint(-60,-60, 2000);
			intake.move_voltage(0);
			break;
	}
	
	// pros::delay(200);
	// leftMG.move_voltage(-4000);
	// rightMG.move_voltage(-4000);
	// pros::delay(1500);
	// leftMG.move_voltage(0);
	// rightMG.move_voltage(0);
	// // driveDistance(-16, 5000);
	
	// clamp.set_value(!clampToggle);
	// pros::delay(250);
	// conveyor.move_voltage(-10000);
	// turnAngle(-67.5);  // Left side 112.5, -67.5 Right side
	// intake.move_voltage(-12000);
	// driveDistance(36, 1000);
	// turnAngle(-110);
	// intake.move_voltage(12000);
	// driveDistance(30, 500);
	

	// leftMG.move_voltage(12000);
	// rightMG.move_voltage(12000);
	// pros::delay(1000);
	// leftMG.move_voltage(0);
	// rightMG.move_voltage(0);
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
	x = abs(x);
	return x == 0 // If x is 0
	? 0
	: x == 1 // If x is 1
	? 1
	: x < 0.5 ? pow(2, 20 * x - 10) / 2
	: (2 - pow(2, -20 * x + 10)) / 2;
}

void opcontrol() {
    while (true) {
		intake.move_voltage(0);
		pose = chassis.getPose();
		printf("X: %f, Y: %f, Theta: %f \n", pose.x, pose.y, pose.theta);
        // // get left y and right x positions
        // float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)/127;
        // float rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/127;

		float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // // move the robot
        // chassis.arcade(easeInOutExpo(leftY) * util::sgn(leftY) * 127, easeInOutExpo(rightX) * util::sgn(rightX) * 127, false, 0.75);
		chassis.arcade(leftY, rightX);
	
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move_voltage(12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move_voltage(-12000);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			clamp.set_value(!clampToggle);
			clampToggle = !clampToggle;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			if (conveyorToggle == 1){
				conveyorToggle = 0;
			} else {
				conveyorToggle = 1;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			if (conveyorDirection == 1){
				conveyorDirection = -1;
			} else {
				conveyorDirection = 1;
			}
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			hangUp = !hangUp;
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			doinkerToggle = !doinkerToggle;
			doinker.set_value(doinkerToggle);
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			if (hangDown){
				hangDown = !hangDown;
			} else {
				if (hangUp){
					hangUp = false;
				}
				hangDown = true;
			}
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			autonomous();
		}

		climbUp.set_value(hangUp);
		climbDown.set_value(hangDown);

		conveyor.move_voltage(-11000 * conveyorToggle * conveyorDirection);

		pros::delay(25);
	}
}
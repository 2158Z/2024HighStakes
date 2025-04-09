#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/motors.h"
#include "misc/screen.h"
#include "util.h"
#include "PID.h"

ASSET(wp1_txt);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMG({-1, -2, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMG({11, 13, 12}, pros::MotorGearset::blue);
pros::MotorGroup ladyBrown({10, -19}, pros::MotorGearset::rpm_200, pros::v5::MotorUnits::degrees);
pros::Motor conveyor(4, pros::MotorGearset::rpm_200);
pros::Motor intake(16, pros::MotorGearset::rpm_200);
pros::MotorGroup leftTracking({-2}, pros::MotorGearset::blue);
pros::MotorGroup rightTracking({11}, pros::MotorGearset::blue);

lemlib::Pose pose = lemlib::Pose(0, 0);

pros::adi::DigitalOut clampIn('C');
pros::adi::DigitalOut clampOut('B');
pros::adi::DigitalOut doinker('A');
pros::adi::DigitalOut climb1('G');
pros::adi::DigitalOut climb2('H');

pros::Rotation lbSensor(15);
pros::Optical optical(6);

int lbTarget = 0;
int lbCurAngle = 0;
int lbMax = 0;

int conveyorToggle = false;
int clampToggle = false;
int conveyorDirection = 1;
bool doinkerToggle = false;

bool colorSorter = false;

pros::Rotation verticalSensor(15);												   // Vertical Sensor
std::vector<float> driveConstants = {6000, 0.17, 0.0005, 1, 2, 75, 0.25, 2000};	   // 1.25
std::vector<float> turnConstants = {12000, 0.015, 0.00, 0.103, 2, 75, 0.75, 2000}; //.0075

lemlib::Drivetrain drivetrain( // Width 12.5, Length 11
	&leftMG,
	&rightMG,
	12.5,						// Track width
	lemlib::Omniwheel::NEW_275, // Wheel type
	450,						// RPM
	0							// Horizontal Drift
);

pros::IMU imu(8);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	3,	 // derivative gain (kD)
	3,	 // anti windup
	0.5, // small error range, in inches
	100, // small error range timeout, in milliseconds
	1.5, // large error range, in inches
	500, // large error range timeout, in milliseconds
	0	 // maximum acceleration (slew)
);

// lemlib::ControllerSettings lateral_controller(
// 	10, // proportional gain (kP)
// 	0, // integral gain (kI)
// 	3, // derivative gain (kD)
// 	0, // anti windup
// 	0, // small error range, in inches
// 	0, // small error range timeout, in milliseconds
// 	0, // large error range, in inches
// 	0, // large error range timeout, in milliseconds
// 	0 // maximum acceleration (slew)
// );

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	10,	 // derivative gain (kD)
	3,	 // anti windup
	1,	 // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3,	 // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0	 // maximum acceleration (slew)
);

lemlib::TrackingWheel leftSideTracking(
	&leftTracking,
	lemlib::Omniwheel::NEW_275,
	-7.5,
	450);

lemlib::TrackingWheel rightSideTracking(
	&rightTracking,
	lemlib::Omniwheel::NEW_275,
	7.5,
	450);

lemlib::OdomSensors sensors(
	&leftSideTracking, // vertical tracking wheel 1, set to null
	&rightSideTracking,
	nullptr,
	nullptr,
	&imu // inertial sensor
);

lemlib::Chassis chassis(
	drivetrain,			// drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors				// odometry sensors
						// &throttleCurve,
						// &steerCurve
);

void driveVoltage(float leftVoltage, float rightVoltage)
{
	leftMG.move_voltage(leftVoltage);
	rightMG.move_voltage(rightVoltage);
}

void driveDistance(float distance, float timeout, std::vector<float> dConstants = driveConstants)
{
	PID leftPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);
	PID rightPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);
	PID trackingWheel(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);

	// leftMG.tare_position();
	// rightMG.tare_position();
	int counter = 0;
	// float lastRightOutput = 0;
	// float lastLeftOutput = 0;
	// float lastRightAddition = 0;
	// float lastLeftAddition = 0;
	float trackingStart = verticalSensor.get_position();

	while (!leftPID.is_settled() || !rightPID.is_settled())
	{
		// float leftTraveled = (leftMG.get_position() / 360) * M_PI * 3.25 * .75;
		// float rightTraveled = (rightMG.get_position() / 360) * M_PI * 3.25 * .75;
		float trackingWheelTraveled = (verticalSensor.get_position() - trackingStart / 360) * M_PI * 2.75;

		// float rightError = distance - rightTraveled;
		// float leftError = distance - leftTraveled;
		float error = distance - trackingWheelTraveled;

		// float leftOutput = leftPID.compute(leftError) * 10000;
		// float rightOutput = rightPID.compute(rightError) * 10000;
		float output = trackingWheel.compute(error) * 10000;

		// leftOutput = util::clampIn(leftOutput, -driveConstants[0], driveConstants[0]);
		// rightOutput = util::clampIn(rightOutput, -driveConstants[0], driveConstants[0]);
		output = util::clamp(output, -driveConstants[0], driveConstants[0]);

		// if(rightOutput > (lastRightOutput + lastRightAddition)) {
		//     rightOutput = lastRightOutput + lastRightAddition;
		//     lastRightAddition += 100;
		// }
		// if(leftOutput > lastLeftOutput + lastLeftAddition) {
		//     leftOutput = lastLeftOutput + lastLeftAddition;
		//     lastLeftAddition += 100;
		// }

		driveVoltage(output, output);

		// lastRightOutput = rightOutput;
		// lastLeftOutput = leftOutput;

		// printf("%f %f %f %f \n", leftError, rightError, leftOutput, rightOutput);
		counter++;
		delay(10);
	}
	driveVoltage(0, 0);
	printf("%s", "settled");
}

void turnAngle(float angle, std::vector<float> tConstants = turnConstants)
{ // relative
	PID turnPID(angle, tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);
	float relativeHeading = 0;
	float absHeading = imu.get_heading();
	float previousAbsHeading = absHeading;
	while (!turnPID.is_settled())
	{
		float deltaAngle = 0;
		absHeading = imu.get_heading();

		deltaAngle = absHeading - previousAbsHeading;
		if (deltaAngle < -180 || deltaAngle > 180)
		{ // if it crosses from 0 to 360 or vice versa
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

void color_sort()
{
	pros::c::optical_rgb_s_t rgb_value;
	while (colorSorter)
	{
		conveyor.move_voltage(12000);
		rgb_value = optical.get_rgb();
		if (rgb_value.blue > 0)
		{
			pros::delay(100);
			conveyor.move_voltage(0);
			pros::delay(100);
			conveyor.move_voltage(12000);
		}
	}
}

void initialize()
{
	LVGL_screen::main();
	chassis.calibrate();
	// lbSensor.reset();
	leftMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	ladyBrown.set_zero_position(0);
	ladyBrown.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize()
{
}

lemlib::MoveToPointParams defaultMoveParams = {.maxSpeed = 90};
lemlib::TurnToPointParams defaultTurnParams = {.maxSpeed = 90};

void autonomous()
{
	pros::Task colorSort(color_sort,"colorSort");
	switch (LVGL_screen::autonID * LVGL_screen::side)
	{
	case 1: // Blue Side Goal Rush
		chassis.setPose(55, -63, 270);
		chassis.moveToPoint(16.5, -53.54, 1500, {.earlyExitRange = 2});
		chassis.turnToHeading(290, 100, {.minSpeed = 100, .earlyExitRange = 2}, false);
		doinker.set_value(true);
		pros::delay(250);
		chassis.turnToHeading(180, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 100}, false);
		doinker.set_value(false);
		pros::delay(250);
		chassis.moveToPoint(16.5, -33, 2000, {.forwards = false, .maxSpeed = 60});
		clampIn.set_value(false);
		clampOut.set_value(true);
		pros::delay(500);
		chassis.moveToPoint(28.5, -20, 3000, {.forwards = false, .maxSpeed = 60}, false);
		clampIn.set_value(true);
		clampOut.set_value(false);
		conveyor.move_voltage(12000);
		intake.move_voltage(-12000);
		chassis.moveToPoint(24, -49, 1500, {.maxSpeed = 80});
		return;
	case 2: // Blue Side 4 Ring
		chassis.setPose(62.5, 47.909, 270);
		chassis.moveToPoint(48, 48, 2000, defaultMoveParams);
		clampIn.set_value(false);
		clampOut.set_value(true);
		chassis.turnToPoint(18, 18, 2000, {.forwards = false, .maxSpeed = 80});
		chassis.moveToPoint(18, 18, 2000, {.forwards = false, .maxSpeed = 80}, false);
		pros::delay(50);
		clampIn.set_value(true);
		clampOut.set_value(false);
		pros::delay(50);
		conveyor.move_voltage(12000);
		intake.move_voltage(-12000);
		chassis.moveToPoint(25, 55, 2000, defaultMoveParams, false);
		chassis.moveToPoint(9, 50, 2000, defaultMoveParams, false);
		chassis.turnToPoint(7.5, 28, 2000, defaultTurnParams, false);
		chassis.moveToPoint(7.5, 28, 2000, defaultMoveParams, false);
		pros::delay(5000);
		return;
	case 3: // Blue Side 4 Ring + Positive Corner Sweep
		chassis.setPose(62.5, 47.909, 270);
		chassis.moveToPoint(48, 48, 2000, defaultMoveParams);
		clampIn.set_value(false);
		clampOut.set_value(true);
		chassis.turnToPoint(18, 18, 2000, {.forwards = false, .maxSpeed = 80});
		chassis.moveToPoint(18, 18, 2000, {.forwards = false, .maxSpeed = 80}, false);
		pros::delay(50);
		clampIn.set_value(true);
		clampOut.set_value(false);
		pros::delay(50);
		conveyor.move_voltage(12000);
		intake.move_voltage(-12000);
		chassis.moveToPoint(25, 51.5, 2000, defaultMoveParams, false);
		chassis.moveToPoint(7, 50, 2000, defaultMoveParams, false);
		chassis.turnToPoint(7, 32, 2000, defaultTurnParams, false);
		chassis.moveToPoint(11, 24, 2000, defaultMoveParams, false);
		chassis.moveToPoint(52, 52, 2000, defaultMoveParams, false);
		doinker.set_value(true);
		pros::delay(500);
		chassis.turnToHeading(270, 2000, {.maxSpeed = 80}, false);
		doinker.set_value(false);
		pros::delay(500);
		chassis.turnToPoint(36, 64, 2000, defaultTurnParams, false);
		chassis.moveToPoint(36, 64, 2000, defaultMoveParams);
		pros::delay(5000);
	}
}

std::vector<float> arcadeControl(double leftInput, double rightInput)
{
	// output voltages of left and right in vector
	std::vector<float> voltages = {0, 0};

	rightInput = util::clamp(rightInput, -0.5, 0.5);

	float max = std::max(fabs(leftInput), fabs(rightInput));
	float difference = leftInput - rightInput;
	float total = leftInput + rightInput;

	if (leftInput >= 0)
	{
		if (rightInput >= 0)
		{
			voltages = std::vector<float>{max, difference};
		}
		else
		{
			voltages = std::vector<float>{total, max};
		}
	}
	else
	{
		if (rightInput >= 0)
		{
			voltages = std::vector<float>{total, -max};
		}
		else
		{
			voltages = std::vector<float>{-max, difference};
		}
	}

	return voltages;
}

float easeInOutExpo(float x)
{
	x = abs(x);
	return x == 0 // If x is 0
			   ? 0
			   : x == 1 // If x is 1
					 ? 1
					 : x < 0.5 ? pow(2, 20 * x - 10) / 2
							   : (2 - pow(2, -20 * x + 10)) / 2;
}

void nextState()
{
	if (lbTarget == 0)
	{
		lbTarget = 27;
		lbMax = 3000;
	}
	else if (lbTarget == 27)
	{
		lbTarget = 135;
		conveyor.move_voltage(9000);
		pros::delay(150);
		conveyor.move_voltage(0);
		lbMax = 2750;
	}
	else if (lbTarget == 135)
	{
		lbTarget = 0;
		lbMax = 6000;
	}
	return;
}

void ladyBrownControl()
{
	printf("Angle: %d, Target: %d \n", lbSensor.get_angle() / 100, lbTarget);
	lbCurAngle = lbSensor.get_angle() / 100;
	if (lbCurAngle > 300 || lbCurAngle < 0)
	{
		lbCurAngle = 0;
	}
	double kP = 0.035;
	double error = lbTarget - lbCurAngle;
	ladyBrown.move_voltage(error * kP * 4000);
}

void opcontrol()
{
	while (true)
	{
		pose = chassis.getPose();
		// printf("X: %f, Y: %f, Theta: %f \n", pose.x, pose.y, pose.theta);
		// // get left y and right x positions
		// float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)/127;
		// float rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/127;

		float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		float rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		conveyor.move_voltage(0);
		intake.move_voltage(0);
		// // move the robot
		// chassis.arcade(easeInOutExpo(leftY) * util::sgn(leftY) * 127, easeInOutExpo(rightX) * util::sgn(rightX) * 127, false, 0.75);
		chassis.tank(leftY, rightY, true);

		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT))
		{
			climb1.set_value(true);
			climb2.set_value(true);
		}
		else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
		{
			climb1.set_value(false);
			climb2.set_value(false);
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake.move_voltage(-12000);
			conveyor.move_voltage(12000);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake.move_voltage(12000);
			conveyor.move_voltage(-12000);
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			clampOut.set_value(true);
			clampIn.set_value(false);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			clampOut.set_value(false);
			clampIn.set_value(true);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			ladyBrown.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			ladyBrown.move_absolute(82, 127);
		}
		else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			ladyBrown.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			ladyBrown.move_absolute(330, 127);
		}
		else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			ladyBrown.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			ladyBrown.move_absolute(0, 127);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			doinker.set_value(true);
		}
		else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			doinker.set_value(false);
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			autonomous();
		}
		pros::delay(25);
	}
}
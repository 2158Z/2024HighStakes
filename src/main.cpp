#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/motors.h"
#include "misc/screen.h"
#include "util.h"
#include "PID.h"
ASSET(wp1_txt);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMG({-1, -3, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMG({11, 13, 12}, pros::MotorGearset::blue);
pros::MotorGroup ladyBrown({10, -8}, pros::MotorGearset::rpm_200);
pros::Motor conveyor(4, pros::MotorGearset::blue);

lemlib::Pose pose = lemlib::Pose(0, 0);

pros::adi::DigitalOut clampIn('B');
pros::adi::DigitalOut clampOut('C');
pros::adi::DigitalOut doinker('A');

pros::Rotation lbSensor(15);
int lbTarget = 0;
int lbCurAngle = 0;
int lbMax = 0;

int conveyorToggle = false;
int clampToggle = false;
int conveyorDirection = 1;
bool doinkerToggle = false;

pros::Rotation verticalSensor(15);												   // Vertical Sensor
std::vector<float> driveConstants = {6000, 0.17, 0.0005, 1, 2, 75, 0.25, 1000};	   // 1.25
std::vector<float> turnConstants = {12000, 0.015, 0.00, 0.103, 2, 75, 0.75, 1000}; //.0075

lemlib::Drivetrain drivetrain(
	&leftMG,
	&rightMG,
	12.5,						// Track width
	lemlib::Omniwheel::NEW_275, // Wheel type
	450,						// RPM
	2							// Horizontal Drift
);

pros::IMU imu(19);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	3,	 // derivative gain (kD)
	3,	 // anti windup
	1,	 // small error range, in inches
	100, // small error range timeout, in milliseconds
	3,	 // large error range, in inches
	500, // large error range timeout, in milliseconds
	0	 // maximum acceleration (slew)
);

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
	&leftMG,
	2.75,
	10.5,
	450 // Offset
);

lemlib::TrackingWheel rightSideTracking(
	&rightMG,
	2.75,
	10.5,
	450 // Offset
);

lemlib::OdomSensors sensors(
	&leftSideTracking, // vertical tracking wheel 1, set to null
	nullptr,
	nullptr,
	nullptr, 
	&imu	 // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(
	3,	  // joystick deadband out of 127
	10,	  // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(
	3,	  // joystick deadband out of 127
	10,	  // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
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

void initialize()
{
	LVGL_screen::main();
	chassis.calibrate();
	lbSensor.reset();
	leftMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMG.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	ladyBrown.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
	switch (LVGL_screen::autonID * LVGL_screen::side)
	{
	case 1: // Blue Right - Three point
		chassis.setPose(50, 16, 90);
		chassis.moveToPoint(32, 16, 2000, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 2}); // changed from 31 to 33
		// chassis.turnToPoint(-24,-24, 2000, {.forwards=false});
		chassis.moveToPoint(18, 26, 2000, {.forwards = false, .maxSpeed = 60});
		while (chassis.isInMotion())
		{
			pros::delay(10); // don't consume all the cpu's resources
		}
		clampIn.set_value(true);
		pros::delay(250);
		conveyor.move_voltage(-12000);
		pros::delay(500);
		// chassis.turnToPoint(24, 48, 2000);
		// chassis.moveToPoint(24, 46, 2000); // changed from 20 to 24
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// delay(250);
		// chassis.turnToPoint(8, 44, 4000, {.minSpeed = 80});
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// chassis.moveToPoint(5, 44, 2000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// conveyor.move_voltage(-12000);
		// pros::delay(2000);
		// chassis.moveToPoint(20, 0, 1000, {.forwards = false});
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		break;
	case 2: // Blue Left
		chassis.setPose(50, -16, 90);
		chassis.moveToPoint(32, -16, 2000, {.forwards = false, .earlyExitRange = 2}); // changed from 31.5 to 34.5
		// chassis.turnToPoint(-24,-24, 2000, {.forwards=false});
		chassis.moveToPoint(16, -28, 4000, {.forwards = false, .maxSpeed = 60});
		while (chassis.isInMotion())
		{
			pros::delay(10); // don't consume all the cpu's resources
		}
		clampIn.set_value(true);
		pros::delay(250);
		conveyor.move_voltage(-12000);
		pros::delay(500);
		// chassis.turnToPoint(24, -48, 2000);
		// chassis.moveToPoint(22, -48, 2000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// chassis.turnToPoint(24, 0, 4000);
		// chassis.moveToPoint(24, 0, 4000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// pros::delay(1000);
		break;
	case -1: // Red Right
		chassis.setPose(-50, -16, 270);
		chassis.moveToPoint(-32, -16, 2000, {.forwards = false, .earlyExitRange = 2}); // changed from 31.5 to 34.5
		// chassis.turnToPoint(-24,-24, 2000, {.forwards=false});
		chassis.moveToPoint(-18, -28, 4000, {.forwards = false, .maxSpeed = 60});
		while (chassis.isInMotion())
		{
			pros::delay(10); // don't consume all the cpu's resources
		}
		clampIn.set_value(true);
		pros::delay(250);
		conveyor.move_voltage(-12000);
		pros::delay(500);
		// chassis.turnToPoint(-24, -48, 2000);
		// chassis.moveToPoint(-22, -48, 2000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// chassis.turnToPoint(-24, 0, 4000);
		// chassis.moveToPoint(-24, 0 ,4000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// pros::delay(1000);
		break;
	case -2: // Red Left
		chassis.setPose(-50, 16, 270);
		chassis.moveToPoint(-32, 16, 2000, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 2}); // changed from 31 to 33
		// chassis.turnToPoint(-24,-24, 2000, {.forwards=false});
		chassis.moveToPoint(-18, 26, 2000, {.forwards = false, .maxSpeed = 60});
		while (chassis.isInMotion())
		{
			pros::delay(10); // don't consume all the cpu's resources
		}
		clampIn.set_value(true);
		pros::delay(250);
		conveyor.move_voltage(-12000);
		pros::delay(500);
		// chassis.turnToPoint(-24, 48, 2000);
		// chassis.moveToPoint(-24, 46, 2000); // changed from 20 to 24
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// delay(250);
		// chassis.turnToPoint(-8, 44, 4000, {.minSpeed = 80});
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// chassis.moveToPoint(-5, 44, 2000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// conveyor.move_voltage(-12000);
		// pros::delay(2000);
		// chassis.turnToPoint(-20, 0, 1000);
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		// chassis.moveToPoint(-20, 0, 1000, {.forwards = false});
		// while (chassis.isInMotion())
		// {
		// 	conveyor.move_voltage(-12000);
		// 	clampIn.set_value(true);
		// 	pros::delay(10);
		// }
		break;
	case 0:
		// leftMG.move_voltage(-4000);
		// rightMG.move_voltage(-4000);
		// pros::delay(350);
		// leftMG.move_voltage(0);
		// rightMG.move_voltage(0);
		// clampToggle = true;
		// pros::delay(250);
		// intake.move_voltage(-12000);
		// conveyor.move_voltage(-12000);
		// chassis.setPose(-50, -22.5, 300);
		// chassis.turnToPoint(-21.5, -24.5, 1500);
		// chassis.moveToPoint(-21.5, -24.5, 1500);
		// chassis.turnToPoint(-23.5, -50, 1500);
		// chassis.moveToPoint(-23.5, -50, 1500);
		// chassis.turnToPoint(-50, -60, 1500);
		// chassis.moveToPoint(-50, -60, 1500);
		// chassis.turnToPoint(-45, -45, 1500);
		// chassis.moveToPoint(-45, -45, 1500);
		// chassis.turnToPoint(-60, -48, 1500);
		// chassis.moveToPoint(-60, -48, 1500);
		break;
		// skills
	}
	// pros::delay(200);
	// leftMG.move_voltage(-4000);
	// rightMG.move_voltage(-4000);
	// pros::delay(1500);
	// leftMG.move_voltage(0);
	// rightMG.move_voltage(0);
	// // driveDistance(-16, 5000);

	// clampIn.set_value(!clampToggle);
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
	if (lbTarget == 0){
		lbTarget = 27;
		lbMax = 3000;
	} else if (lbTarget == 27){
		lbTarget = 135;
		conveyor.move_voltage(9000);
		pros::delay(150);
		conveyor.move_voltage(0);
		lbMax = 2750;
	} else if (lbTarget == 135){
		lbTarget = 0;
		lbMax = 6000;
	}
	return;
}

void ladyBrownControl()
{
	printf("Angle: %d, Target: %d \n", lbSensor.get_angle()/100, lbTarget);
	lbCurAngle = lbSensor.get_angle() / 100;
	if (lbCurAngle > 300 || lbCurAngle < 0) {lbCurAngle = 0;}
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
		float rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// // move the robot
		// chassis.arcade(easeInOutExpo(leftY) * util::sgn(leftY) * 127, easeInOutExpo(rightX) * util::sgn(rightX) * 127, false, 0.75);
		chassis.arcade(leftY, rightX);

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			clampIn.set_value(!clampToggle);
			clampOut.set_value(clampToggle);
			clampToggle = !clampToggle;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			if (conveyorToggle == 1)
			{
				conveyorToggle = 0;
			}
			else
			{
				conveyorToggle = 1;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			if (conveyorDirection == 1)
			{
				conveyorDirection = -1;
			}
			else
			{
				conveyorDirection = 1;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			doinkerToggle = !doinkerToggle;
			doinker.set_value(doinkerToggle);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			autonomous();
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			nextState();
		}

		conveyor.move_voltage(-12000 * conveyorToggle * conveyorDirection);
		ladyBrownControl();

		pros::delay(25);
	}
}
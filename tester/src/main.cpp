#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
using namespace okapi;

void autonomous() {
	std::shared_ptr<OdomChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors({1, 2}, {-3, -4})
    .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 10_in}, imev5GreenTPR})
    .withSensors(
        ADIEncoder{'A', 'B'}, // Left encoder in ADI ports A & B
        ADIEncoder{'C', 'D', true}  // Right encoder in ADI ports C & D (reversed)
	)
	.withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
    .buildOdometry();

	std::shared_ptr<AsyncMotionProfileController> profileControllerF = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();

	// frontChassis->setState({0_in, 0_in, 0_deg});
	// frontChassis->driveToPoint({1_ft, 1_ft});
	// frontChassis->turnToAngle(90_deg);
	// frontChassis->turnToPoint({5_ft, 0_ft});

  	// profileControllerF->generatePath(
    // 	{{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 0_deg}}, "A");
  	// profileControllerF->setTarget("A");
  	// profileControllerF->waitUntilSettled();
	profileControllerF->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 90_deg}}, "B");
	profileControllerF->setTarget("B", true, true);
  	profileControllerF->waitUntilSettled();
	chassis->turnToAngle(90_deg);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

// using namespace pros;
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_front_mtr(1);
	pros::Motor right_front_mtr(3, true);
	pros::Motor left_back_mtr(2);
	pros::Motor right_back_mtr(4, true);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		double left = master.get_analog(ANALOG_LEFT_Y);
		double right = master.get_analog(ANALOG_RIGHT_Y);

		left_front_mtr.move_velocity(left);
		left_back_mtr.move_velocity(left);
		right_front_mtr.move_velocity(right);
		right_back_mtr.move_velocity(right);

		// double forward = master.get_analog(ANALOG_LEFT_Y);
		// double rotate = master.get_analog(ANALOG_RIGHT_X);

		// left_front_mtr.move_velocity(forward + rotate);
		// left_back_mtr.move_velocity(forward + rotate);
		// right_front_mtr.move_velocity(forward - rotate);
		// right_back_mtr.move_velocity(forward - rotate);

		pros::delay(10);
	}
}
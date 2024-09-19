#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -20});
pros::MotorGroup right_mg({1, 10});
pros::Imu inertial(2);

// change based on drivetrain
// const double driveTicksPerInch = 1/(3.25*3.1415)*5/3*300;
const double driveTicksPerInch = 48.9708;

// turn pid function
void turn(double heading) {
    double angle = fmod(heading - inertial.get_heading(), 360);
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    double targetAngle = inertial.get_rotation() + angle;

    // constants to tune
    double kP = 90;
    double kI = 4;
    double kD = 200;
    double integral = 0;
    double lastError = targetAngle - inertial.get_rotation();

    int restedStates = 0;
    int stalledStates = 0;
    while (restedStates < 5 && stalledStates < 50) {
        double error = targetAngle - inertial.get_rotation();
        if (fabs(error) < 30) integral += error;
        if (fabs(error) < 0.1) integral = 0;

        if (fabs(error - lastError) < 0.005) stalledStates++;
        else stalledStates = 0;

        double out = kP * error + kI * integral + kD * (error - lastError);
        left_mg.move_voltage(out);
        right_mg.move_voltage(-out);

        lastError = error;

        pros::delay(10);

        if (fabs(targetAngle - inertial.get_rotation()) < 2) restedStates++;
        else restedStates = 0;
    }
    left_mg.move_voltage(0);
    right_mg.move_voltage(0);
}

// move pid function
void move(double distance) {
    left_mg.tare_position();
    right_mg.tare_position();

    distance *= driveTicksPerInch;

    // constants to tune
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double integral = 0;
    double lastError = distance;

    int restedStates = 0;
    int stalledStates = 0;
    while (restedStates < 5 && stalledStates < 50) {
        double error = (distance - (left_mg.get_position() + right_mg.get_position())/2);
        if (fabs(error) < 2.5*driveTicksPerInch) integral += error;
        if (fabs(error) < 0.05*driveTicksPerInch) integral = 0;

        if (fabs(error - lastError) < 0.05*driveTicksPerInch) stalledStates++;
        else stalledStates = 0;

        double out = kP * error + kI * integral + kD * (error - lastError);
        left_mg.move_voltage(out);
        right_mg.move_voltage(out);

        lastError = error;

        pros::delay(10);

        if (fabs(distance - (left_mg.get_position() + right_mg.get_position())/2) < 0.1*driveTicksPerInch) restedStates++;
        else restedStates = 0;
    }
    left_mg.move_voltage(0);
    right_mg.move_voltage(0);
}

// move pid that keeps straight
void move_straight(double distance) {
	double initialHeading = inertial.get_rotation();
    left_mg.tare_position();
    right_mg.tare_position();

    distance *= driveTicksPerInch;

	// movement constants
    double kP = 30;
    double kI = 0;
    double kD = 0;
    double integral = 0;
    double lastError = distance;

	// turning constants
    double hkP = 90;
    double hkI = 4;
    double hkD = 200;
    double hintegral = 0;
    double hlastError = 0;

    int restedStates = 0;
    int stalledStates = 0;
    while ((restedStates < 5 && stalledStates < 50)) {
        double error = (distance - (left_mg.get_position() + right_mg.get_position())/2);
        if (fabs(error) < 2.5*driveTicksPerInch) integral += error;
        if (fabs(error) < 0.05*driveTicksPerInch) integral = 0;

        if (fabs(error - lastError) < 0.05*driveTicksPerInch) stalledStates++;
        else stalledStates = 0;

        double herror = initialHeading - inertial.get_rotation();
        if (fabs(herror) < 30) hintegral += herror;
        if (fabs(herror) < 0.5) hintegral = 0;

        double out = kP * error + kI * integral + kD * (error - lastError);
        double hOut = hkP * herror + hkI * hintegral + hkD * (herror - hlastError);
        double maxOut = fmax(12000, fmax(out - hOut, out + hOut));
        left_mg.move_voltage((out + hOut)*12000/maxOut);
        right_mg.move_voltage((out - hOut)*12000/maxOut);

        lastError = error;
		hlastError = herror;

        pros::delay(10);

        if (fabs(distance - (left_mg.get_position() + right_mg.get_position())/2) < 0.1*driveTicksPerInch) restedStates++;
        else restedStates = 0;
    }
    left_mg.move_voltage(0);
    right_mg.move_voltage(0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	inertial.reset(true);
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
void autonomous() {}

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
void opcontrol() {
    controller.clear();

	while (true) {
		if (controller.get_digital_new_press(DIGITAL_A)) {
			left_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
			right_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
            controller.print(0, 0, "                             ");
            
            double start = pros::millis();
            // code


            controller.print(0, 0, "%.0f                             ", (pros::millis() - start));
            pros::delay(500);
            controller.print(1, 0, "%.2f                             ", inertial.get_heading());
            pros::delay(500);
		}

		left_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
		right_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
		int power = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X);
		left_mg.move(power + turn*.7);
		right_mg.move(power - turn*.7);
		pros::delay(10);
	}
}

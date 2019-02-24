#include "robot-config.h"
vex::line LineSensor(Brain.ThreeWirePort.D);

// ---------------------------------------------------------------------------------- //
// ---------------------------GLOBALS, CONSTANTS, & MACROS--------------------------- //
// ---------------------------------------------------------------------------------- //

#define SLEEP_MS(x) task::sleep(x);

const int HZ_50_WAIT = 1000/50;  // this is 20ms
const int HZ_25_WAIT = 1000/50*2;  // this is 40ms
const int HZ_15_WAIT = 1000/15;  // this is 66.6...ms

const int VISION_WIDTH = 312;
const int VISION_HEIGHT = 210;
const int VISION_MAX_DISTANCE = 30000;
const int VISION_OFFSET = 0;  // 0 px.

const int RED = 0;
const int BLUE = 1;

// Global scope variables.
int g_flywheelPower = 0;
int g_ticks = 0;

// ---------------------------------------------------------------------------------- //
// ---------------------------------UTILITY FUNCTIONS-------------------------------- //
// ---------------------------------------------------------------------------------- //

// absolute value function, i.e: x = |x|;
float abs(float num) { if (num > 0) { return num; } else { return -num; } }

// Forces an abs() number between abs() the min and max values. Note: Keeps the sign of the number
double capMinMax(double val, double min, double max) {
    if (val < min && val > 0) { val = min; } else if (val > -min && val < 0) { val = -min; }
    if (val > max) { val = max; } else if (val < -max) { val = -max; }
    return val;
}

// The wheel's circumfrench is pi*4 inches
float inToRev(float inches) { return inches / (3.14159265358979323*4); }

// ---------------------------------------------------------------------------------- //
// ------------------------------DRIVE CONTROL FUNCTIONS----------------------------- //
// ---------------------------------------------------------------------------------- //

// Set the left and right motor to spin forward using the controller Axis values as the velocity value.
void setLeftRightDriveSpeed(int leftSpeed=0, int rightSpeed=0) {
    LeftDrive1.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
    LeftDrive2.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
    RightDrive1.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
    RightDrive2.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
}

// This function drives each side independently.  Used for non-gyro rotation. Stops control.
void driveForLeftRight(float leftRevolutions, float rightRevolution, int speed) {
    LeftDrive1.rotateFor(leftRevolutions, rotationUnits::rev, speed, velocityUnits::pct, false);
    LeftDrive2.rotateFor(leftRevolutions, rotationUnits::rev, speed, velocityUnits::pct, false);
    RightDrive1.rotateFor(rightRevolutions, rotationUnits::rev, speed, velocityUnits::pct, false);
    RightDrive2.rotateFor(rightRevolutions, rotationUnits::rev, speed, velocityUnits::pct, true);
}

// Drives straight for inches at speed
void driveFor(float inches, int speed=85) {
    LeftDrive1.rotateFor(inToRev(inches), rotationUnits::rev, speed, velocityUnits::pct, false);
    LeftDrive2.rotateFor(inToRev(inches), rotationUnits::rev, speed, velocityUnits::pct, false);
    RightDrive1.rotateFor(inToRev(inches), rotationUnits::rev, speed, velocityUnits::pct, false);
    RightDrive2.rotateFor(inToRev(inches), rotationUnits::rev, speed, velocityUnits::pct, true);
}

// ---------------------------------------------------------------------------------- //
// -----------------------------SENSOR CONTROL FUNCTIONS----------------------------- //
// ---------------------------------------------------------------------------------- //

// This uses the light sensor to detect balls and turn the intake off.
const int LIGHT_LIMIT = 67;
int IntakeUntil() {
    // Query line sensor value and compare it to the limit.
    while(LineSensor.value(percentUnits::pct)>LIGHT_LIMIT) {
        IntakeMotor1.spin(directionType::rev, 100, velocityUnits::pct);
        SLEEP_MS(HZ_50_WAIT);
    }

    IntakeMotor1.spin(directionType::rev, 0, velocityUnits::pct);
    IntakeMotor1.stop();

    Brain.Screen.print("LOG: Intake Light Triggered");
    Brain.Screen.newLine();
    return 0;
}

// Drives until it hits a certain sonar value. Uses A P loop.
// NOTE: sonar works best with moving small distances or fixing drift error in long auto runs.
void driveUntilSonar(float sonarValue, float maxPower=55, float minPower=2, int timeOut=600) {
    // initialize P loop variables
	float kp = 3.75; // TODO: tune this.
	float error;
	float drivePower = 0;

    // Init timer stuff.
    int ticksElapsed = 0;
    int timeTicks = 0;

	// finish check variables
	bool atTarget = false;

	// initialize gyro data variables
	float targetReading = sonarValue;

	// run motors until target is within 1 degree certainty
	while (!atTarget && (timeTicks < timeOut)) {
		error = Sonar.distance(distanceUnits::in) - targetReading; 	// calculate error  //-30
		drivePower = error * kp;  // calculate PD loop output  //speed

		// Keep speed between min and max power.
		drivePower = capMinMax(drivePower, minPower, maxPower);

		// Send power to motors and add mods (scaled with drive power).
		setLeftRightDriveSpeed(drivePower, drivePower);

		// check for finish
		if (abs(error) > 1) // if robot is within 1f inch of target and timer flag is off
			ticksElapsed = 0;  // Reset ticks elapsed

		if (ticksElapsed > 4)  // if the timer is over for 80ms (4 ticks) and timer flag is true
			atTarget = true;  // set boolean to complete while loop

        ticksElapsed++;
        timeTicks++;
		SLEEP_MS(HZ_25_WAIT);  //let motors update. (slower hz so less frequent sensor sampling)
	}
	setLeftRightDriveSpeed();  // Reset motors.

    Brain.Screen.print("LOG: Sonar Move Done");
    Brain.Screen.newLine();
    /*Kimi no sei, Kimi no sei, Kimi no sei de*/
}

// Standard PID loop to turn to target deg. For turning gyroscope.
void rotateTo (float targetDegrees, float maxPower=70, float minPower=1, int timeOut=6000/20) {
	// initialize PID loop variables
	float kp = 1.80f;
    float ki = -0.0014f;
    float kd = 0.1f;

	float error; float drivePower;
    float integral; float derivative; float prevError;

    // Init timer stuff.
    int ticksElapsed = 0;
    int timeTicks = 0;

	// finish check variables
	bool atTarget = false;

	// initialize gyro data variables
	float targetReading = -targetDegrees;  // No relative rotation pls, thanks.

	// run motors until target is within 1 degree certainty
	while (!atTarget && (timeTicks < timeOut)) {
        error = targetDegrees - GyroSensor.value(rotationUnits::deg);  // Get new error from sensor.

        // Update PID values
        integral = integral + error;
        if(error == 0) { integral = 0; }
        derivative = error - prevError;
        prevError = error;

        drivePower = error*kp+integral*ki+derivative*kd;  // Combine pid variables.

        // Cap and assign speed.
        drivePower = capMinMax(drivePower, minPower, maxPower);
        setLeftRightDriveSpeed(drivePower, -drivePower);

		// Check if error is at sufficient value.
		if (abs(error) > 1) // if robot is within 1 inch of target and timer flag is off
			ticksElapsed = 0;  // Reset ticks elapsed

        // Check if the robot is at target for long enough.
		if (ticksElapsed > 2)  // if the timer is over for 40ms (2 ticks) and timer flag is true
			atTarget = true;

        ticksElapsed++;
        timeTicks++;
		SLEEP_MS(HZ_50_WAIT);  //let motors update.
	}
	setLeftRightDriveSpeed(0, 0);  // Reset motors.

    Brain.Screen.print("LOG: Gryo Rotate Done");
    Brain.Screen.newLine();
    /*Kimi no sei dayou-*/
 }

// This function pauses the program and just shoots the ball.
void shootBall() {
    ShootingMotor1.rotateFor(4, rotationUnits::rev, 100, velocityUnits::pct, false);
    ShootingMotor2.rotateFor(-4, rotationUnits::rev, 100, velocityUnits::pct, true);
}

int intakeIn() {
    while(true) { IntakeMotor1.spin(directionType::rev, 100, velocityUnits::pct); SLEEP_MS(HZ_50_WAIT); }
    return 1;
}

int intakeOut() {
    while(true) { IntakeMotor1.spin(directionType::fwd, 100, velocityUnits::pct); SLEEP_MS(HZ_50_WAIT); }
    return 1;
}

// This stops ANY intake task.
int intakeOff() {
    vex::task::stop( intakeOut ); vex::task::stop( intakeIn );
    IntakeMotor1.stop(vex::brakeType::brake);
    return 0;
}

// This function requires a ball to be in the barrel and one to be in hold.
void doubleShot() {
    shootBall();  // first shot

    vex::task iu1( intakeIn );
    ArmMotor.rotateFor(0.787, rotationUnits::rev, 100, velocityUnits::pct, false);
    SLEEP_MS(100);

    shootBall();  // second shot
    intakeOff();
    ArmMotor.rotateFor(-0.787, rotationUnits::rev, 100, velocityUnits::pct);

    Brain.Screen.print("LOG: Double Shot Done");
    Brain.Screen.newLine();
}

// This is a way to concurrently shoot while moving by triggering tasks to be started in (n)ms.
int g_waitTime = 0;
float g_amountRotate = 0.0f;
int waitUntilRotateShootingMotorFor() {
    SLEEP_MS(g_waitTime);
    ShootingMotor2.rotateFor(-g_amountRotate, rotationUnits::rev, 100, velocityUnits::pct, true);
    ShootingMotor1.rotateFor(g_amountRotate, rotationUnits::rev, 100, velocityUnits::pct, true);
    return 0;
}

// This function rotates the canon a certain number of rotations, but immediately
// gives control to the next function.
void triggerRotateCannon(float revolutions) {
    ShootingMotor1.rotateFor(revolutions, rotationUnits::rev, 100, velocityUnits::pct, false);
    ShootingMotor2.rotateFor(-revolutions, rotationUnits::rev, 100, velocityUnits::pct, false);
}

// ---------------------------------------------------------------------------------- //
// ------------------------------VISION SENSOR FUNCTIONS----------------------------- //
// ---------------------------------------------------------------------------------- //

// This function gets the x position of the object from the vision sensor,
int getVisionObjX( int colour ) {
    if (colour == RED)
        Vision.takeSnapshot(SIG_R);
    else
        Vision.takeSnapshot(SIG_B);

    int objCount = Vision.objectCount;

    // Find the object closest to the center.
    int closestDistance = 30000;
    for (int i=0; i<objCount; i++) {
        if (Vision.objects[i].width > 4 && Vision.objects[i].height > 4) {  // Ignore tiny objects
            // Find distance from center.
            int disFromCenter = (VISION_WIDTH/2) - Vision.objects[i].centerX;
            if (abs(disFromCenter) < abs(closestDistance)) {
                closestDistance = disFromCenter;
            }
        }
    }

    return closestDistance;
}

// This is the camera code.
void aimRobot(int colour) {
    // initialize P loop variables
    float maxPower = 50; float minPower = 2;
    int timeOut = 4000/67;  // 4s
    float kp = 0.095f;  // This is the kp. Change this down to make oscillation & speed smaller.

    // Init timer stuff.
    int ticksElapsed = 0;
    int timeTicks = 0;

    float error = 0;
	float drivePower = 0;

	// finish check variables
	bool atTarget = false;

    // run motors until target is within n px certainty
	while (!atTarget && (timeTicks < timeOut)) {
        // Find distance from center.
        int disFromCenter = getVisionObjX(colour);
		error = disFromCenter - VISION_OFFSET; 	// calculate error.

        // This means robot didn't find a target; exits.
        if (disFromCenter == VISION_MAX_DISTANCE) {
            atTarget = true;
            Brain.Screen.print("LOG: Vis-Sensor failed to find object");
            Brain.Screen.newLine();
        }

		drivePower = error * kp;  // calculate PD loop output  //speed
		drivePower = capMinMax(drivePower, minPower, maxPower);
		setLeftRightDriveSpeed(-drivePower, drivePower);

		// check for finish
		if (abs(error) > 9)  // If robot is within 9px of target and timer flag is off
			ticksElapsed = 0;  // Reset ticks elapsed

		if (ticksElapsed > 3)  // if the timer is over for 210ms (3 ticks) and timer flag is true
			atTarget = true;  // set boolean to complete while loop

        ticksElapsed++;
        timeTicks++;
		SLEEP_MS(HZ_15_WAIT);  //let motors update.
	}

    setLeftRightDriveSpeed(0, 0);

    Brain.Screen.print("LOG: Centering Complete");
    Brain.Screen.newLine();
}

// ---------------------------------------------------------------------------------- //
// --------------------------------SETUP & AUTONOMOUS-------------------------------- //
// ---------------------------------------------------------------------------------- //

void pre_auton( void ) { }

void autonomous( void ) {
    triggerRotateCannon(2);

    task ii1( intakeIn );
    driveFor(39, 60);  // Goes to pick up the first arena ball.

    SLEEP_MS(750);
    intakeOff();  // Intake turns off after picking up the ball.
    driveFor(-38, 60);  // Drive back

    SLEEP_MS(200);
    driveForLeftRight(-0.845, 0.845, 40);  // Turn left to face flags, row #1.
    driveFor(-13, 40);  // Line up for double shot.

    SLEEP_MS(50);
    VISION_OFFSET = -5; aimRobot(BLUE);  // Aim at blue targets for double shot.
    SLEEP_MS(50);
    doubleShot();  // shoot top two targets

    driveForLeftRight(-0.0331, 0.0331, 40);  // Re-align robot.
    driveFor(52, 80);  // Run into bottom target.
    driveUntilSonar(4.3);  // Use sonar to fix encoder sensor drift.
    SLEEP_MS(350);

    triggerRotateCannon(2);
    driveFor(-92, 90);

    driveForLeftRight(0.9, -0.9, 40);  // Turn right

    // Align with wall
    setLeftRightDriveSpeed(-40, -40);
    SLEEP_MS(1600);
    setLeftRightDriveSpeed(-5, -5);
    SLEEP_MS(300);

    // Get ball # 2.
    setLeftRightDriveSpeed(0,0);
    driveFor(40+10, 60);  // Goes to pick up the ball
    task ii2( intakeIn );
    SLEEP_MS(350);

    driveFor(-33-3, 60);
    intakeOff();  // Intake turns off after picking up the ball

    triggerRotateCannon(2);
    SLEEP_MS(200);

    driveForLeftRight(-0.82, 0.82, 40);
    aimRobot(RED);  // Re-align robot.

    driveFor(60, 60);
    driveForLeftRight(0.853, -0.853, 40);

    driveFor(44.5, 60);
    driveForLeftRight(-0.853, 0.853, 40);

    driveFor(8, 70);

    SLEEP_MS(50);
    aimRobot(BLUE);  // Aim robot for shooting a single ball.
    shootBall();

    driveForLeftRight(-0.07, 0.07, 40);
    driveUntilSonar(3.4); // Re-align robot.
    SLEEP_MS(200);

    driveFor(-20, 50);

    driveForLeftRight(-0.458, 0.458, 40);

    driveFor(-49, 70);
    driveForLeftRight(-0.37, 0.37, 40);

    driveFor(20, 60);
    task ii3( intakeIn );
    SLEEP_MS(1000);
    intakeOff();  // Intake turns off after picking up the ball
    driveFor(-33-3, 80);

    driveForLeftRight(0.851, -0.851, 40);
    driveFor(-51,80);

    driveForLeftRight(-0.8, 0.8, 40);
    setLeftRightDriveSpeed(-50, -40);  // Do a curve maneuver, then re-align against wall.
    SLEEP_MS(1000);
    driveFor(50,60);

    task ii4( intakeIn );
    SLEEP_MS(450);
    intakeOff();  // Intake turns off after picking up the ball
    driveFor(-33, 60);

    driveForLeftRight(0.79, -0.79, 40);
    aimRobot(BLUE);  // Re-Align robot.
    driveFor(23,60);
    VISION_OFFSET = -7; aimRobot(BLUE);  // Aim robot.
    doubleShot();  // Shoots the two right-side flags after collecting the balls.

    driveFor(-5,60);
    driveForLeftRight(0.851, -0.851, 40);
    driveFor(-65,80);  // Finally goes on the platform and wins world championship!
}

// ---------------------------------------------------------------------------------- //
// -------------------------------TELEOPERATED CONTROL------------------------------- //
// ---------------------------------------------------------------------------------- //

int currentTick = 0;
void usercontrol( void ) {
    while (true) {
        // Tank Drive.
        setLeftRightDriveSpeed(Controller1.Axis3.value(), Controller1.Axis2.value());

        // Arm Controls
        if(Controller1.ButtonX.pressing()) {
            ArmMotor.spin(directionType::fwd, 80, velocityUnits::pct);
        } else if(Controller1.ButtonB.pressing()) {
            ArmMotor.spin(directionType::rev, 80, velocityUnits::pct);
        } else {
            ArmMotor.stop(vex::brakeType::brake); // Case: up or down button is not pressed
        }

        // Intake Control
        if(Controller1.ButtonL2.pressing()) {
            IntakeMotor1.spin(directionType::fwd, 100, velocityUnits::pct);
        } else if(Controller1.ButtonL1.pressing()) {
            IntakeMotor1.spin(directionType::rev, 100, velocityUnits::pct);
        } else if(Controller1.ButtonA.pressing()){
            IntakeMotor1.stop(brakeType::brake);
        }

        // Cannon Control
        if(Controller1.ButtonR2.pressing()) {
           shootBall();
        } else if(Controller1.ButtonR1.pressing()) {
            ShootingMotor1.spin(directionType::fwd, 100, velocityUnits::pct);
            ShootingMotor2.spin(directionType::rev, 100, velocityUnits::pct);
        } else if (Controller1.ButtonUp.pressing()){
            doubleShot();
        } else {
            ShootingMotor1.stop(brakeType::brake);
            ShootingMotor2.stop(brakeType::brake);
        }

        SLEEP_MS(20);

        // Update sonar sensor to screen ever 25 ticks (2 hz)
        currentTick++;
        if(currentTick%25 == 0) {
            Controller1.Screen.print("sonar: %d in", Sonar.distance(distanceUnits::in));
        }
    }
}

// ---------------------------------------------------------------------------------- //
// -------------------------------COORDINATOR FUNCTION------------------------------- //
// ---------------------------------------------------------------------------------- //

int main() {
    pre_auton(); // Run the pre-autonomous function.

    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    // Prevent main from exiting with an infinite loop.
    while(true) { SLEEP_MS(100); }
}

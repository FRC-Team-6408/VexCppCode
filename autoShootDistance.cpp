#include "robot-config.h"

const int POLL_SPEED = 1000/50;  // 50 iterations per second.

const int TOP_TARGET = 44;  // height = 44in.
const int MID_TARGET = 30;  // height = 30in.
const double ANGLE = 30;  // the angle of the projectile's initial velocity. (angle is not 30.)
const double SONAR_FLYWHEEL_DIST = 0;  // Distance from point the ball stops touching the flywheel to the front of the sonar sensor.
const double FG_VANCOUVER = -9.83249;  // Change this number when no longer in vancouver.  // Use standard value?
const double VI = 10;  // in in/s (speed is not 10in/s)

// This function uses physics 12 math to calculate the needed distance from the wall.
// The variable "VI" is the calculated speed of the ball.
void setAimPosition( int targetHeight ) {
    Brain.Screen.print("shootTarget ~ start");

    // Find needed components.
    double dx = Sonar.distance(distanceUnits::in) + SONAR_FLYWHEEL_DIST;  // Get distance from wall to sonar, then sonar to flywheel.
    double dy = targetHeight;

    // Calculate the horizontal distance.
    // Also, this equation is split into multiple parts so it is easy to read.
    double front = (10/FG_VANCOUVER*100)*cos(ANGLE)*cos(ANGLE);
    double pt1 = sqrt(2) * sqrt( VI*VI*(50*VI*VI*tan(ANGLE)*tan(ANGLE) - ( (FG_VANCOUVER*-1*100)*dy*(1/(cos(ANGLE)*cos(ANGLE))) ) ));
    double pt2 = 10*VI*VI*tan(ANGLE);
    double dxOut = abs(front * (pt1 - pt2))

    double driveDistance = (dxOut - dx);

    // TODO: move driveDistance inches.

    Brain.Screen.print("shootTarget ~ end");
}

void pre_auton( void ) { }

// The auto function.  Called once.
void autonomous( void ) { }

// Some tiny director functions.
void HitTop (void) { shootTarget(TOP_TARGET); }
void HitMid (void) { shootTarget(MID_TARGET); }

// User control task - The driver loop.
void usercontrol( void ) {
    // Bind controller functions for automatic flywheel shots.
    Controller1.ButtonA.pressed(HitTop);
    Controller1.ButtonB.pressed(HitMid);

    // User control code here, inside the loop
    while (true) {
        // TODO: insert drive and control code here.

        //Sleep the task for a short amount of time to prevent wasted resources.
        vex::task::sleep(POLL_SPEED);
    }
}

// Main will set up the competition functions and callbacks.
int main() {
    //Run the pre-autonomous function.
    pre_auton();

    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.
    while(true) {
      vex::task::sleep(100);  //Sleep the task for a short amount of time to prevent wasted resources.
    }
}

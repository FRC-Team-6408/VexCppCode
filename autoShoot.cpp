#include "robot-config.h"

//Creates a competition object that allows access to Competition methods.
//vex::competition    Competition; // FOR SOME REASON IT IS GETTING ANGRY AT THIS LINE AND I DONT KNOW WHY.

const int POLL_SPEED = 1000/50;  // 50 iterations per second.

const int TOP_TARGET = 44;  // height = 44in.
const int MID_TARGET = 30;  // height = 30in.
const double SPEED_MOD = 0.5;  // smaller means less power.
const double ANGLE = 30;  // the angle of the projectile's initial velocity. (angle is not 30.)
const double SONAR_FLYWHEEL_DIST = 0;  // Distance from point the ball stops touching the flywheel to the front of the sonar sensor.
const double FG_VANCOUVER = -9.83249;  // Change this number when no longer in vancouver.  // Use standard value?

// This function uses physics 12 math to calculate the initial velocity of the ball.
// The variable "SPEED_MOD" is used to convert in/s to flywheel speed.
void shootTarget( int targetHeight ) {
    Brain.Screen.print("shootTarget ~ start");

    // Find needed components.
    double dx = Sonar.distance(distanceUnits::in) + SONAR_FLYWHEEL_DIST;  // Get distance from wall to sonar, then sonar to flywheel.
    double dy = targetHeight;

    // Calculate the initial velocity.
    // Also, this equation is split into multiple parts so it is easy to read.
    double top = (FG_VANCOUVER/2) * pow(dx, 2.0);
    double bot = pow(cos(ANGLE), 2.0) * ( dy - (dx*sin(ANGLE) / cos(ANGLE)) );
    double vi = sqrt( top / bot );

    // Convert to flywheel percent.
    double percentSpeed = vi*SPEED_MOD;

    // Rotate motor for 0.25s.
    Flywheel.spin(directionType::fwd, percentSpeed, velocityUnits::pct);
    vex::task::sleep(250);

    // Stop flywheel.
    Flywheel.spin(directionType::fwd, 0, velocityUnits::pct);

    Brain.Screen.print("shootTarget ~ end");
}

void pre_auton( void ) {
    // Calculations?
}

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
        // Control the main drive.
        double speedL = (Controller1.Axis3.value() + Controller1.Axis4.value()) / 2;
        double speedR = (Controller1.Axis3.value() - Controller1.Axis4.value()) / 2;
        DriveL.spin(directionType::fwd, speedL, velocityUnits::pct);
        DriveR.spin(directionType::fwd, speedR, velocityUnits::pct);

        // Control the Barrel intake.
        if (Controller1.ButtonL1.pressing() == true) {
            Barrel.spin(directionType::fwd,100,velocityUnits::pct);
        } else if (Controller1.ButtonL2.pressing() == true) {
            Barrel.spin(directionType::rev,100,velocityUnits::pct);
        } else {
            Barrel.spin(directionType::fwd,0,velocityUnits::pct);
        }

        // Control the Flywheel.
        if (Controller1.ButtonR1.pressing() == true) {
            Flywheel.spin(directionType::fwd, 100, velocityUnits::pct);
        } else if (Controller1.ButtonR2.pressing() == true) {
            Flywheel.spin(directionType::rev, 100, velocityUnits::pct);
        } else {
            Flywheel.spin(directionType::fwd,0,velocityUnits::pct);
        }

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

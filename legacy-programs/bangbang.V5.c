#include "robot-config.h"

#define SLEEP_MS(n) vex::task::sleep(n)
#define FORWARDS vex::directionType::fwd
#define IN_PERCENT vex::velocityUnits::pct

// These units are in rpm.  //TODO: figure out rpm.
const int TOP_FLAG = 23000;
const int BOT_FLAG = 19000;

// Global scope variables.
int g_flywheelPower = 0;
int g_flywheelRPM = 0;

// This function updates the rpm every 20 ms (50 times per second.)
int UpdateRPM() {
	while(true) {
        g_flywheelRPM = FlyWheel.velocity(vex::velocityUnits::rpm);
        vex::task::sleep(20);
	}
}

// This function checks for buttons pressed.
int CheckInput ( void ) {
    // TODO: change buttons.
    while (true) {
        if (Controller1.ButtonX.pressing()) {  // Shit's pressed, make this a toggle pls. -- /*WHY SO VULGAR?*/
            (g_flywheelRPM < TOP_FLAG) ? g_flywheelPower=100 : g_flywheelPower=0;
        } else if (Controller1.ButtonY.pressing()) {
            (g_flywheelRPM < BOT_FLAG) ? g_flywheelPower=100 : g_flywheelPower=0;
        }
        SLEEP_MS(5); // TODO: make this wait larger?
    }
}

int UpdateFlywheel() {
	while(true) {
		FlyWheel.spin(FORWARDS, g_flywheelPower, IN_PERCENT);
		SLEEP_MS(10);
	}
}

void usercontrol( void ) {
    // Start tasks.
    vex::task rpm( UpdateRPM );
    vex::task inp( CheckInput );
    vex::task fly( UpdateFlywheel );

    //RightDrive2.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);

    while(true) { SLEEP_MS(100); }  // Stop usercontrol from exiting.
}

// Don't need this right now.
void pre_auton( void ) { }
void autonomous( void ) {
    SLEEP_MS(1500);  // Gyro init.
    Controller1.Screen.print("start auto skills");
}

int main() {
    pre_auton();

    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    // Prevent main from exiting with an infinite loop.
    while(true) { SLEEP_MS(100); }
}

#include "robot-config.h"
#include <string>  // holy shit i can include stuff.
#include <cmath>

typedef std::string string;  // This makes the string class easier to use.

const int BRAIN_WIDTH = 480;  // in px
const int BRAIN_HEIGHT = 240;  // in px
const int BRAIN_ROW_COUNT = 12;  //TODO: make this the bottom most row.
const int BRAIN_ITEM_PADDING = 8;  // in px

const int STD_HZ = 50;
const int STD_WAIT = 1000/STD_HZ;  // This is 20ms

// absolute value
double abs(double n) { if (n < 0.0) { return -n; } else { return n; } }

// This does some basic systems reporting to the brain.
void initialize( void ) {
    Brain.Screen.print("########  Systems Report:  ########");
    Brain.Screen.newLine();

    bool sdCardStatus = Brain.SDcard.isInserted();
    Brain.Screen.print("sdcard inserted = %i", sdCardStatus);
    Brain.Screen.newLine();

    vex::devices dev;  // This is a local variable
    Brain.Screen.print("devices connected = %i", dev.number());
    Brain.Screen.newLine();

    Brain.Screen.setCursor(BRAIN_ROW_COUNT, 1);
    Brain.Screen.print("Press the (A) button or touch the screen to continue...");

    int ticks = 0;
    bool exit = false;
    // This loop waits for the controller or screen to be pressed before exiting.
    // A small dot animation plays to show user brain is not crashed.
    while(!exit) {
        if(Controller.ButtonA.pressing() || Brain.Screen.pressing()) {
            exit = true;
        }

        // Change the bottom line when needed for the dot animation.
        if (ticks % STD_HZ*4 == 0) {
            Brain.Screen.clearLine(BRAIN_ROW_COUNT, vex::color::black);
            Brain.Screen.print("Press the (A) button or touch the screen to continue...");
        } else if (ticks % STD_HZ*4 == STD_HZ || ticks % STD_HZ*4 == STD_HZ*3) {
            Brain.Screen.clearLine(BRAIN_ROW_COUNT, vex::color::black);
            Brain.Screen.print("Press the (A) button or touch the screen to continue..");
        } else if (ticks % STD_HZ*4 == STD_HZ*2) {
            Brain.Screen.clearLine(BRAIN_ROW_COUNT, vex::color::black);
            Brain.Screen.print("Press the (A) button or touch the screen to continue.");
        }

        vex::task::sleep(STD_WAIT);
        ticks++;
    }
}

void motorSettings ( void ) { }

void thermalInfo ( void ) {

}

void sdReader ( void ) {

}

void visionViewer ( void ) {

}

void autoWriter ( void ) {

}

void drawGif( void ) { }

int gRowSize = 2;  // This is how many items can be in one row.

// The Button Item class contains the information for the each different screen item / action.
// (i.e. test motors, system report, etc...)
class ButtonItem {
private:
    static int _idGenerator;  // This generates a new id every time a new object is created.
public:
    int m_id;  // This is the position relative to the other objects (i.e. 0 is top, 1 is under it.)
    string m_title;  // The name of the Button.
    void (*m_funcPtr)();  // This is a pointer the button's function.

    ButtonItem(string title, void (*funcPtr)()) {
        this->m_funcPtr = funcPtr;
        this->m_title = title;

        _idGenerator++;
        this->m_id = _idGenerator;
    }

    // Get drawing index.
    int GetX() { return m_id%gRowSize; }
    int GetY() { return std::floor(m_id/gRowSize); }
    void Draw() {
        //todo: THIS
        Brain.Screen.drawRectangle(GetX(), GetY(), GetItemSize(), GetItemSize(), vex::color::black);
    }

    static int GetItemSize() {
        int verticalItems = (gRowSize <= 1) ? 1 : std::floor(gRowSize/2);
        return BRAIN_HEIGHT/verticalItems - BRAIN_ITEM_PADDING*2;
    }
    static int getMaxYIndex() { return std::floor(ButtonItem::_idGenerator/gRowSize); }
};
int ButtonItem::_idGenerator=-1;

// Button Item / "Program" declarations.
ButtonItem SystemReport = ButtonItem("System Report", initialize);
ButtonItem MotorSettings = ButtonItem("Motor Settings", motorSettings);
ButtonItem ThermalInformation = ButtonItem("Thermal Information", thermalInfo);
ButtonItem SDCardViewer = ButtonItem("SD Card Viewer", sdReader);
ButtonItem VisionViewer = ButtonItem("Vision Sensor Viewer", visionViewer);
ButtonItem EditAutoCode = ButtonItem("Edit Autonomous Code", autoWriter);
ButtonItem DrawGif = ButtonItem("Show Logo", drawGif);

// This function starts the menu.  This is the main loop of the program.
int xindex = 0; int yindex = 0;  // This is the coords of the cursor.
bool requestNewFrame = true;

bool upToggle;  bool downToggle;
void menu ( void ) {
    // Whenever a change is made and a new frame is requested
    bool exit = false;
    while(!exit) {
        if (Controller.ButtonUp.pressing()) {
            if(!upToggle) {
                upToggle = true;
                yindex--;
                if (yindex < 0) {yindex = 0;}
            }
        } else if(!Controller.ButtonUp.pressing()) {
            upToggle = false;
        }

        if (Controller.ButtonDown.pressing()) {
            if(!upToggle) {
                upToggle = true;
                yindex++;
                if (yindex > ButtonItem::getMaxYIndex()) {yindex = ButtonItem::getMaxYIndex();}
            }
        } else if(!Controller.ButtonUp.pressing()) {
            upToggle = false;
        }

        if (requestNewFrame) {
            requestNewFrame = false;
            Brain.Screen.clearScreen(vex::color::white);
            SystemReport.Draw();
            MotorSettings.Draw();
            ThermalInformation.Draw();
            SDCardViewer.Draw();
            VisionViewer.Draw();
            EditAutoCode.Draw();
            DrawGif.Draw();
        }

        vex::task::sleep(STD_WAIT);
    }
}



int main() {
    initialize();  // Check the status of the brain and connections.
    menu();  // Enter menu after status check;

    while(true) { vex::task::sleep(STD_WAIT*5); }

}

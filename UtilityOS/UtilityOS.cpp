typedef signed int int32_t;typedef unsigned int uint32_t;typedef unsigned char uint8_t;typedef unsigned long uint64_t;namespace vex{enum percentUnits{pct };enum timeUnits{sec,msec};enum currentUnits{amp};enum voltageUnits{volt,mV};enum powerUnits{watt};enum torqueUnits{Nm,InLb};enum rotationUnits{deg,rev,raw};enum velocityUnits{pct,rpm,dps};enum distanceUnits{mm,in,cm};enum analogUnits{pct,range8bit,range10bit,range12bit,mV};enum directionType{fwd,rev};enum brakeType{coast,brake,hold};enum gearSetting{ratio36_1,ratio18_1,ratio6_1};enum fontType{mono10,mono20,mono40,mono60,prop20,prop30,prop40,prop60,mono15,mono12};enum triportType{analogInput,analogOutput,digitalInput,digitalOutput,button,potentiometer,lineSensor,lightSensor,gyro,accelerometer,motor,servo,quadEncoder,sonar,motorS};enum controllerType{primary,partner};class triport{public:typedef char port;};class accelerometer{public:accelerometer::accelerometer(triport::port & port);int32_t value(percentUnits units);void changed(void (*)(void));};class analog_in{public:analog_in::analog_in(triport::port &port);int32_t value(percentUnits units);void changed(void (*)(void));};class brain{class brain::battery{public:uint32_t capacity(percentUnits units = percentUnits::pct);double temperature(percentUnits units = percentUnits::pct);};class brain::lcd{public:void setCursor(int32_t row, int32_t col);void setFont(fontType font);void setPenWidth(uint32_t width);void setOrigin(int32_t x, int32_t y);int32_t column();int32_t row();void setPenColor(const color &color);void setPenColor(int hue);void setPenColor(const char *color);void setFillColor(const color &color);void setFillColor(int hue);void setFillColor(const char *color);void print(const char *format, ...);void printAt(int32_t x, int32_t y, const char *format, ...);void clearScreen(void);void clearScreen(int hue);void clearScreen(const char *color);void clearScreen(const color &color);void clearLine(void);void clearLine(int hue);void clearLine(const char *color);void clearLine(const color &color);void newLine(void);void drawPixel(int x, int y);void drawLine(int x1, int y1, int x2, int y2);void drawRectangle(int x, int y, int width, int height);void drawRectangle(int x, int y, int width, int height, int hue);void drawRectangle(int x, int y, int width, int height, const char *color);void drawRectangle(int x, int y, int width, int height, const color &color);void drawCircle(int x, int y, int radius);void drawCircle(int x, int y, int radius, int hue);void drawCircle(int x, int y, int radius, const char *color);void drawCircle(int x, int y, int radius, const color &color);void pressed(void (*callback)(void));void released(void (*callback)(void));int32_t x_position();int32_t y_position();bool pressing();bool render();bool render(bool bVsyncWait, bool bRunScheduler = true);};class brain::sdcard{public:bool isInserted();int32_t loadfile(const char *name, uint8_t *buffer, int32_t len);int32_t savefile(const char *name, uint8_t *buffer, int32_t len);int32_t appendfile(const char *name, uint8_t *buffer, int32_t len);};lcd Screen = lcd();battery Battery = battery();sdcard SDcard = sdcard();brain::brain();timer Timer = timer();};class bumper{public:bumper::bumper(triport::port &port);int32_t pressing();void pressed(void (*callback)(void));void released(void (*callback)(void));};class color{public:color::color(int value);color::color(uint8_t r, uint8_t g, uint8_t b);bool isTransparent() const;color hsv(uint32_t hue, double sat, double value);color web(const char *color);};class competition{public:bool bStopTasksBetweenModes;competition::competition();void autonomous(void (*callback)(void));void drivercontrol(void (*callback)(void));bool isEnabled();bool isDriverControl();bool isAutonomous();bool isCompetitionSwitch();bool isFieldControl();};class controller{public:class axis{public:void changed(void (*callback)(void));int32_t value(void);int32_t position(percentUnits units);};class button{public:void pressed(void (*callback)(void));void released(void (*callback)(void));bool pressing(void);};class lcd{public:void setCursor(int32_t row, int32_t col);void print(const char *format, ...);void clearScreen(void);void clearLine(void);void clearLine(int number);void newLine(void);};button ButtonL1 = button();button ButtonL2 = button();button ButtonR1 = button();button ButtonR2 = button();button ButtonUp = button();button ButtonDown = button();button ButtonLeft = button();button ButtonRight = button();button ButtonA = button();button ButtonB = button();button ButtonX = button();button ButtonY = button();axis Axis1 = axis();axis Axis2 = axis();axis Axis3 = axis();axis Axis4 = axis();lcd Screen = lcd();controller(controllerType id);};class digital_in{public:digital_in(triport::port &port);int32_t value();void high(void (*callback)(void));void low(void (*callback)(void));};class digital_out{public:digital_out(triport::port &port);int32_t value();void set(bool value);};class encoder{public:encoder(triport::port &port);void resetRotation(void);void setRotation(double val, rotationUnits units);double rotation(rotationUnits units);double velocity(velocityUnits units);void changed(void (*callback)(void));};class gyro{public:gyro(triport::port &port);int32_t value(rotationUnits units);int32_t value(percentUnits units);int32_t value(analogUnits units);void startCalibration(int32_t value = 0);bool isCalibrating();void changed(void (*callback)(void));};class light{public:light(triport::port &port);int32_t value(percentUnits units);int32_t value(analogUnits units);void changed(void (*callback)(void));};class limit{public:limit(triport::port &port);int32_t pressing();void pressed(void (*callback)(void));void released(void (*callback)(void));};class line{public:line(triport::port &port);int32_t value(percentUnits units);int32_t value(analogUnits units);void changed(void (*callback)(void));};class motor{public:motor(int32_t index);motor(int32_t index, bool reverse);motor(int32_t index, gearSetting gears);motor(int32_t index, gearSetting gears, bool reverse);void setReversed(bool value);void setVelocity(double velocity, velocityUnits units);void setStopping(brakeType mode);void resetRotation(void);void setRotation(double value, rotationUnits units);void setTimeout(int32_t time, timeUnits units);void spin(directionType dir);void spin(directionType dir, double velocity, velocityUnits units);void rotateTo(double rotation, rotationUnits units, bool waitForCompletion = true);void rotateTo(double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion = true);void rotateFor(double rotation, rotationUnits units, bool waitForCompletion = true);void rotateFor(double time, timeUnits units);void rotateFor(double time, timeUnits units, double velocity, velocityUnits units_v);void rotateFor(double rotation, rotationUnits units, double velocity, velocityUnits units_v, bool waitForCompletion = true);void startRotateTo(double rotation, rotationUnits units);void startRotateTo(double rotation, rotationUnits units, double velocity, velocityUnits units_v);void startRotateFor(double rotation, rotationUnits units);void startRotateFor(double rotation, rotationUnits units, double velocity, velocityUnits units_v);bool isSpinning();void stop(void);void stop(brakeType mode);void setMaxTorque(double value, percentUnits units);void setMaxTorque(double value, torqueUnits units);void setMaxTorque(double value, currentUnits units);directionType direction(void);double rotation(rotationUnits units);double velocity(velocityUnits units);double current(currentUnits units);double power(powerUnits units);double torque(torqueUnits units);double efficiency(percentUnits units);double temperature(percentUnits units);};class motor29{public:motor29(triport::port &port);motor29(triport::port &port, bool reverse);void setVelocity(double velocity, percentUnits units);void setReversed(bool value);void spin(directionType dir);void spin(directionType dir, double velocity, velocityUnits units);void stop(void);};class motor_victor{public:motor_victor(triport::port &port);motor_victor(triport::port &port, bool reverse);void setVelocity(double velocity, percentUnits units);void setReversed(bool value);void spin(directionType dir);void spin(directionType dir, double velocity, velocityUnits units);void stop(void);};class mutex{public:void lock();bool try_lock();void unlock();};class pot{public:pot(triport::port &port);int32_t value(rotationUnits units);int32_t value(percentUnits units);int32_t value(analogUnits units);void changed(void (*callback)(void));};class pwm_out{public:pwm_out(triport::port &port);void state(int32_t value, percentUnits units);};class semaphore{public:void lock();void lock(uint32_t time);void unlock();bool owner();};class servo{public:servo(triport::port &port);void servo(triport::port &port);void position(double value, rotationUnits units);};class sonar{public:sonar(triport::port &port);double distance(distanceUnits units);void changed(void (*callback)(void));};class task{public:task(int (*callback)(void));task(int (*callback)(void), int32_t priority);void stop();void stop(int (*callback)(void));void suspend();void resume();int32_t priority();void setPriority(int32_t priority);void sleep(uint32_t time);void yield();};class thread{public:thread(int (*callback)(void));thread(void (*callback)(void));int32_t get_id();void join();void detach();bool joinable();void *native_handle();void swap(thread &__t);void swap(thread &__x, thread &__y);void interrupt();void setPriority(int32_t priority);int32_t priority();int32_t hardware_concurrency();};class timer{uint32_t time() const;uint32_t time(timeUnits units) const;void clear();uint32_t system();uint64_t systemHighResolution();void event(void (*callback)(void), uint32_t value);void event(void (*callback)(void *), uint32_t value);};};

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

const int STD_PEN_WIDTH = 8;
const int BOLD_PEN_WIDTH = 16;

// Check this.
const int FONT_HEIGHT = 20;  // in px
const int FONT_WIDTH = 10;  // in px

// absolute value
double abs(double n) { if (n < 0.0) { return -n; } else { return n; } }

// returns a the amount of elements split.  Splits a string into multiples.
// REMEMBER TO DELETE sOut after using it.
int splitString(string s, string* strOut) {
    string delimiter = " ";

    int size = std::count(s.begin(), s.end(), ' ');
    strOut = new string[size];

    int index=0;
    size_t pos = 0;
    string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        strOut[index] = token;  // Assign token to array element.
        s.erase(0, pos + delimiter.length());
        index++;
    }

    return size;
}

// This loop waits for the controller or screen to be pressed before exiting.
// A small dot animation plays to show user brain is not crashed.
void pause() {
    int ticks = 0;
    bool exit = false;
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

// This does some basic systems reporting to the brain.
void initialize( void ) {
    Brain.Screen.setCursor(1, 1);
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

    exitLoop();
}

void motorSettings ( void ) { }

void thermalInfo ( void ) {
}

// Give user command line access to the sd card.
void sdReader ( void ) {
    // First give a blurb saying if there even is an sd card.
    Brain.Screen.clearScreen(vex::color::black);
    Brain.Screen.setCursor(1, 1);

    bool sdCardStatus = Brain.SDcard.isInserted();
    Brain.Screen.print("sdcard inserted = %i", sdCardStatus);
    Brain.Screen.newLine();

    pause();
    if (!sdCardStatus) { return; }  // If no sd card, exit.
    // TODO: let user iteract with files etc...
}

void visionViewer ( void ) {
}

void autoWriter ( void ) {
}

void drawGif( void ) {
    //  loop:
    //      string str = "thing" + number +"i.png";
    //      Brain.Screen.drawImageFromFile(str, xpos, ypos);
}

int gRowSize = 3;  // This is how many items can be in one row.

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

    // Draw this item to the brain.
    void Draw() {
        // Find position based on index.
        int xloc = GetX()*GetItemSize() + (GetX()+1)*BRAIN_ITEM_PADDING;
        int yloc = GetY()*GetItemSize() + (GetY()+1)*BRAIN_ITEM_PADDING;
        int size = GetItemSize();

        // Draw border rectangle at needed thickness & color, case: if selected.
        if (xindex == GetX() && yindex == GetY()) {
            Brain.Screen.setPenWidth(BOLD_PEN_WIDTH);
            Brain.Screen.drawRectangle(xloc, yloc,  size, size, vex::color::yellow);
            Brain.Screen.setPenWidth(STD_PEN_WIDTH);
        } else {
            Brain.Screen.drawRectangle(xloc, yloc,  size, size, vex::color::black);
        }

        // TODO: VERIFY THIS FACT: monospaced characters are 1/2 the width of the height of a char.
        //                         the 20 in mono20 refers to 20px in height.

        // Determine how many lines should be drawn for the title.
        int titleLen = m_title.length;
        if (titleLen*FONT_WIDTH < GetItemSize()) {  // case: draw a single line.
            Brain.Screen.printAt(xloc, yloc, m_title);
        } else {  // case: draw each word in a new line.
            string* titlePtr;
            int arrSize = splitString(m_title, titlePtr);
            for(int i=0; i<arrSize; i++) {
                Brain.Screen.printAt(xloc, yloc+(FONT_HEIGHT+1)*i, titlePtr[i]);
            }

            delete[] titlePtr;  // titlePtr must be deleted because new was used.
        }

    }

    // Get drawing index.
    int GetX() { return m_id%gRowSize; }
    int GetY() { return std::floor(m_id/gRowSize); }

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

// menu variables.
int xindex = 0; int yindex = 0;  // This is the coords of the cursor.
bool requestNewFrame = true;

// This function has all the menu controll checking.
bool upToggle;  bool downToggle;
void menuControl( void ) {
    if (Controller.ButtonUp.pressing()) {
        if(!upToggle) {
            upToggle = true;
            {  // Action when button is first pressed:
                yindex--;
                if (yindex < 0) {yindex = 0;}
                requestNewFrame = true;
            }
        }
    } else if(!Controller.ButtonUp.pressing() && upToggle) {
        upToggle = false;
    }

    if (Controller.ButtonDown.pressing()) {
        if(!downToggle) {
            downToggle = true;
            {  // Action when button is first pressed:
                yindex++;
                if (yindex > ButtonItem::getMaxYIndex()) {yindex = ButtonItem::getMaxYIndex();}
                requestNewFrame = true;
            }
        }
    } else if(!Controller.ButtonUp.pressing() && downToggle) {
        downToggle = false;
    }

}

// This function starts the menu.  This is the main loop of the program.
void menu ( void ) {
    Brain.Screen.setPenColor(vex::color::black);  // Make all text black in the menu.

    // Whenever a change is made and a new frame is requested
    bool exit = false;
    while(!exit) {
        menuControl();  // Check for movement.

        // Redraw the screen when the "cursor" is moved.
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

void setup() {
    // This is the initial font used everywhere in the program.
    Brain.Screen.setFont(vex::fontType::mono20);
    Brain.Screen.setPenWidth(STD_PEN_WIDTH);
}

int main() {
    setup();  // This function sets constants & stuff.  Do not call this again.

    initialize();  // Check the status of the brain and connections.
    menu();  // Enter menu after status check;

    while(true) { vex::task::sleep(STD_WAIT*5); }

}

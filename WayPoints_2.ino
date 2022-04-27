/* Touch works, just with orientation "touch" is above the "ON" button
 * All but HOLY HILL works, gets stuck there
 * Works on blue and red TFT display
 * Added curentWaypoint for function with GPS waypoints
 * Added if ((gps.location.isValid()) && (currentWaypoint == '1'))
 * Added CAPITOL< HOME< CARL & HOLY HILL
 * MEGA must be connected or you will get "Serial1 was not declared in this scope" error
 * MEGA  ***This is the one for Capitol Airport ***
 * Removed Date and Time
 * RED TFT (ILI9341)
 * Pin19  to 4th pin on sensor
 * GPS Sensor GT-U7
 * Displays Compass and GPS headers, SEEMS TO WORK!
 * Changed KPH to MPH
 * Changed Altitude Meters to Feet
 * Added GPS Valid Gren/Red box ***ONLY valid if 
 * GPS SAT signal is lost, not if wire #19 is 
 * pulled out and Arduino is not receiving signals.***
 * Added Dist and Heading to Capitol Airport Waukesha WI (can be changed)
 * Changed SPEED to not show anything behind the decimal point.
 * Deleted GPS TIME It was off by hours and who needs it?
 * Added touch buttons but they arent working, goes with orange block
 * Blocked compass, touch ON or OFF is above the "ON" button
*/

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
static const int RXPin = 18, TXPin = 19;
static const uint32_t GPSBaud = 9600;
#include <Arduino.h>
#define USE_ADAFRUIT_SHIELD_PINOUT 1
#include <Adafruit_GFX.h> 
#include <Adafruit_Sensor.h> 
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

// For buttons **************************
#include <TouchScreen.h>
#define MINPRESSURE 20  // was 200
#define MAXPRESSURE 1000  //Was 1000

TinyGPSPlus gps;          // May have GPS TIME calculations

#include <SPI.h>
#include "Adafruit_ILI9341.h"

// Assign names to common 16-bit color values:
#define   BLACK  0x0000
#define   BLUE   0x001F
#define   RED    0xF800
#define   GREEN  0x07E0
#define   CYAN   0x07FF
#define   YELLOW 0xFFE0
#define   ORANGE 0xFD20
#define   WHITE  0xFFFF

String    Time, Date;
float     NumberSats, Latitude, Longitude, Bearing;
float     AltitudeMETRES, AltitudeMILES, AltitudeKM, AltitudeFEET;
float     SpeedKPH, SpeedMPH, SpeedKNOTS, SpeedMPS;
//float     distanceToWAYPOINT;           /////////////////////////

const int centreX  = 230; // Location of the compass display on screen
const int centreY  = 120;
const int diameter = 70; // Size of the compass
int       dx = centreX, dy = centreY;
int       last_dx = centreX, last_dy = centreY - diameter * 0.85;

// For changing WayPoints
int currentWaypoint = 1;
int CAPITOL         = 1;
int HOME            = 2;
int HOLY_HILL       = 3;
int CARL            = 4;

// ALL Touch panels and wiring is DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x9341   pins for touch screen
const int TS_LEFT=922,TS_RT=93,TS_TOP=79,TS_BOT=903; // Touch screen touch area calibration landscape or portrait? fix this!!!!!!!!!!!!
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_GFX_Button on_btn, off_btn;  //Declaring on and off buttons

int pixel_x, pixel_y;     //Touch_getXY() updates global vars
bool Touch_getXY(void)
 {
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);

    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);

    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
                 }

    return pressed; // needed for button press

  }

// For stats that happen every 5 seconds
unsigned long last = 0UL;

void setup()
 {
  Serial.begin(115200);
  Serial1.begin(9600); // connect gps sensor
  //ss.begin(GPSBaud)

  tft.begin();            // Start the TFT display
  //tft.setRotation(5);     // Rotate screen by 90°
  tft.setTextSize(2);     // Set medium text size
  tft.setTextColor(YELLOW);
  
//*****BEGIN TOUCH SETUP  *******************************************************************************

Serial.begin(115200);
    uint16_t ID = tft.readID();
    Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    Serial.println("Calibrate for your Touch Panel");
    

    //if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.begin(ID);
    tft.setRotation(1);            //Landscape
    tft.fillScreen(BLACK);

    on_btn.initButton(&tft,  180, 150, 80, 100, WHITE, CYAN, BLACK, "ON", 2);   // Moves, draws BUTTON, & touch point, should be portrait for proper operation
    off_btn.initButton(&tft, 260, 150, 80, 100, WHITE, CYAN, BLACK, "OFF", 2);  // And touch position (&tft)
    on_btn.drawButton(false);   // Button will draw if pressed and this is disabled
    off_btn.drawButton(false);
  } 
//*** END TOUCH SETUP  ***********************************************************************************



void loop() {
  
  Latitude       = gps.location.lat();
  Longitude      = gps.location.lng();
  Bearing        = gps.course.deg();
  SpeedMPH       = gps.speed.mph();           // KEEP THIS! No speed!
  SpeedKPH       = gps.speed.kmph();
  NumberSats     = gps.satellites.value();
  AltitudeMETRES = gps.altitude.meters();     // KEEP THIS! No Altitude!
  AltitudeFEET   = gps.altitude.feet();

  

                      // ********* GPS GOOD? This works ONLY if SAT signal is lost NOT if Pin19 is pulled out *******
if (gps.satellites.value ()> 3 )
   tft.fillRect(0, 0, 30, 8 * 4, GREEN); 
else   
   tft.fillRect(0, 0, 30, 8 * 4, RED);     //left,top,right,down * 4? seems to increase the depth X4, Green



// ***************** Adding Dist & Course to CAPITOL *******************************************
                     
if ((gps.location.isValid()) && (currentWaypoint == 1))
  {
    tft.fillRect(32, 0, 202, 5 * 4, BLACK);   // Distance Left, Distance down, Distance right, Distance Down, * ????? See above tft.fillrect
    PrintText(20, 0, "  CAPITOL", WHITE, 2);
  //PrintText(40, 0, "distanceToWAYPOINT:" + String(),2, 2);        //////////////
  //tft.fillRect(45, 40, 90, 19 * 4, BLACK); //LAT, LONG, ALT, SAT area
  
  static const double WAYPOINT_LAT = 43.089619, WAYPOINT_LON = -88.179440;
      double distanceToWAYPOINT =
      TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
      double courseToWAYPOINT = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
      tft.fillRect(135, 0, 120, 15, BLACK);  //Erases old data
      tft.setTextColor(CYAN);
      tft.setCursor(150, 0);                                // Sets it back to not print over "WAYPOINT"
      tft.println(TinyGPSPlus::cardinal(courseToWAYPOINT));   // Prints "cardinal heading (SE) to gps location" on TFT
      tft.setCursor(190, 0);
      tft.println(distanceToWAYPOINT/1000*.6213712, 1);       // Prints Distance to WAYPOINT calculated for miles
      tft.setCursor(250, 0);
      tft.print("Miles");
    Serial.println("sdjhbdjkfhvb");
  }
 

// end Addding Distance & Heading to WAYPOINT

// *** Adding Dist & Course to Home *******************************************
if ((gps.location.isValid()) && (currentWaypoint == 2))
  {
tft.fillRect(32, 0, 202, 5 * 4, BLACK);   // Distance Left, Distance down, Distance right, Distance Down, * ????? See above tft.fillrect
PrintText(20, 0, "  HOME", WHITE, 2);
  //PrintText(40, 0, "distanceToWAYPOINT:" + String(),2, 2);        //////////////
  
  //tft.fillRect(45, 40, 90, 19 * 4, BLACK);   //LAT, LONG, ALT, SAT area
    
  static const double WAYPOINT_LAT = 43.200000, WAYPOINT_LON = -88.300000;
      double distanceToWAYPOINT =
        TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
      double courseToWAYPOINT =
        TinyGPSPlus::courseTo( gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
        
      tft.fillRect(135, 0, 110, 15, BLACK);  //Erases old data over # of miles
      tft.setTextColor(CYAN);
      tft.setCursor(150, 0);                                // Sets it back to not print over "WAYPOINT"
      tft.println(TinyGPSPlus::cardinal(courseToWAYPOINT));   // Prints "cardinal heading (SE) to gps location" on TFT
      tft.setCursor(190, 0);
      tft.println(distanceToWAYPOINT/1000*.6213712, 1);       // Prints Distance to WAYPOINT calculated for miles
      //tft.setCursor(250, 0);
      //tft.print("Miles");
  }
 

// end Addding Distance & Heading to HOME

// *** Adding Dist & Course to CARL *******************************************
if ((gps.location.isValid()) && (currentWaypoint == 3))
  {
tft.fillRect(32, 0, 202, 5 * 4, BLACK);   // Distance Left, Distance down, Distance right, Distance Down, * ????? See above tft.fillrect
PrintText(20, 0, "  CARL", WHITE, 2);
  //PrintText(40, 0, "distanceToWAYPOINT:" + String(),2, 2);        //////////////
  //tft.fillRect(45, 40, 90, 19 * 4, BLACK);   //LAT, LONG, ALT, SAT area
    
  static const double WAYPOINT_LAT = 43.369462, WAYPOINT_LON = -88.592716;
      double distanceToWAYPOINT =
        TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
      double courseToWAYPOINT =
        TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
        
      tft.fillRect(135, 0, 110, 15, BLACK);  //Erases old data
      tft.setTextColor(CYAN);
      tft.setCursor(150, 0);                                // Sets it back to not print over "WAYPOINT"
      tft.println(TinyGPSPlus::cardinal(courseToWAYPOINT));   // Prints "cardinal heading (SE) to gps location" on TFT
      tft.setCursor(190, 0);
      tft.println(distanceToWAYPOINT/1000*.6213712, 1);       // Prints Distance to WAYPOINT calculated for miles
      //tft.setCursor(250, 0);
      //tft.print("Miles");
  }
  
// end Addding Distance & Heading to CARL

// *** Adding Dist & Course to HOLY HILL *******************************************
if ((gps.location.isValid()) && (currentWaypoint == 4))
  {
tft.fillRect(32, 0, 202, 5 * 4, BLACK);   // Distance Left, Distance down, Distance right, Distance Down, * ????? See above tft.fillrect
PrintText(20, 0, " HOLY HILL", CYAN, 2);
  //PrintText(40, 0, "distanceToWAYPOINT:" + String(),2, 2);        //////////////
    //tft.fillRect(45, 40, 90, 19 * 4, BLACK);   //LAT, LONG, ALT, SAT area
    
  static const double WAYPOINT_LAT = 43.244222, WAYPOINT_LON = -88.327722;  // Not HOLY HILL, ulast 6 digits of both used for troubleshooting
      double distanceToWAYPOINT =
      TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);
      double courseToWAYPOINT =
      TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), WAYPOINT_LAT, WAYPOINT_LON);

      tft.setTextColor(CYAN);
      tft.setCursor(150, 0);                                // Sets it back to not print over "WAYPOINT"
      tft.println(TinyGPSPlus::cardinal(courseToWAYPOINT));   // Prints "cardinal heading (SE) to gps location" on TFT
      tft.setCursor(190, 0);
      tft.println(distanceToWAYPOINT/1000*.6213712, 1);       // Prints Distance to WAYPOINT calculated for miles
      //tft.setCursor(235, 0);
      //tft.print("Miles");
  }
 

// end Addding Distance & Heading to Home?   ***************************************************************************


  DisplayGPSdata(NumberSats, Latitude, Longitude, AltitudeMETRES, SpeedKPH, Bearing); // Select units as required
  smartDelay(1000);
  if (millis() > 5000 && gps.charsProcessed() < 10)  Serial.println(F("No GPS data received: check wiring"));

//****** BEGIN TOUCH LOOP *******************************************************************************************

{
    bool down = Touch_getXY();
    //on_btn.press(down && on_btn.contains(pixel_x, pixel_y));
    off_btn.press(down && off_btn.contains(pixel_x, pixel_y));
    delay(150);
    //Serial.print("It reads this line");

    if (on_btn.justReleased())
        on_btn.drawButton();
    if (off_btn.justReleased())
        off_btn.drawButton();
    if (on_btn.justPressed()) {
        on_btn.drawButton(true);
    }
    
// Changing waypoints ##############################################
    if (off_btn.justPressed())   {
        off_btn.drawButton(true);
        currentWaypoint= currentWaypoint + 1;
    }

if(currentWaypoint >= 5)
     { 
      currentWaypoint = 1;       // Waypoints are 1,2,3,4
     }
            // 1=CAPITOL, 2=HOME, 3=HOLY HILL, 4=CARL, 5+ goes back to 1, this sets CAPITOL to read at startup
      }
   }

//****** END TOUCH LOOP *******************************************************************************************


//#####################################################################
  void DisplayGPSdata(float dNumberSats, float dLatitude, float dLongitude, float dAltitude, float dSpeed, float dBearing) {

//  PrintText(20, 0, "  WAYPOINT", CYAN, 2);
//  PrintText(40, 0, "distanceToWAYPOINT:" + String(),2, 2);        //////////////
  tft.fillRect(45, 40, 90, 19 * 4, BLACK);   // blacks out old Lat Lon Alt & Sat data
  PrintText(0, 45, "LAT:" + String(dLatitude), YELLOW, 2);
  PrintText(0, 63, "LON:" + String(dLongitude), YELLOW, 2);
  PrintText(0, 81, "ALT:" + String(dAltitude * 3.28084, 1) + "  ", WHITE, 2);  // Removed FT, didn't need it
  PrintText(0, 99, "SAT:" + String(dNumberSats, 0), YELLOW, 2);
  

//  SPEED
  tft.fillRect(5, 210, 100, 25, BLACK);    // Left side to side, up and down, Right amount long, down for an amount, color (Blacks out old speed)
  tft.setTextColor(WHITE);
  tft.setCursor(10, 180);
  tft.setTextSize(3);
  tft.print("SPEED");
  PrintText(35, 210, "" + String(dSpeed  / 1.609, 0) + "  ", WHITE, 3);  //SPEED

  tft.setCursor(200, 220);
//  Display_Compass(dBearing);
}
//#####################################################################


/* // TEMPORARY ATTEMPT WITH NO COMPASS  **(ALSO UN BLOCK dISPLAY COMPASS ABOVE)***********************************

void Display_Compass(float dBearing) {
  int dxo, dyo, dxi, dyi;
  tft.setCursor(0, 0);
  tft.drawCircle(centreX, centreY, diameter, WHITE); // Draw compass circle
  for (float i = 0; i < 360; i = i + 22.5) {
    dxo = diameter * cos((i - 90) * 3.14 / 180);
    dyo = diameter * sin((i - 90) * 3.14 / 180);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    tft.drawLine(dxo + centreX, dyo + centreY, dxi + centreX, dyi + centreY, WHITE);
  }
  PrintText((centreX - 5), (centreY - diameter - 18), "N", GREEN, 2);
  PrintText((centreX - 5), (centreY + diameter + 5) , "S", GREEN, 2);
  PrintText((centreX + diameter + 5),  (centreY - 5), "E", GREEN, 2);
  PrintText((centreX - diameter - 15), (centreY - 5), "W", GREEN, 2);
  dx = (0.85 * diameter * cos((dBearing - 90) * 3.14 / 180)) + centreX; // calculate X position ( Change dBearing to bearing? to get to WAYPOINT Airport?)
  dy = (0.85 * diameter * sin((dBearing - 90) * 3.14 / 180)) + centreY; // calculate Y position
  draw_arrow(last_dx, last_dy, centreX, centreY, 5, 5, BLACK);   // Erase last arrow
  draw_arrow(dx, dy, centreX, centreY, 5, 5, YELLOW);           // Draw arrow in new position
  last_dx = dx;
  last_dy = dy;
}
//#####################################################################
void draw_arrow(int x2, int y2, int x1, int y1, int alength, int awidth, int colour) {
  float distance;
  int dx, dy, x2o, y2o, x3, y3, x4, y4, k;
  distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
  dx = x2 + (x1 - x2) * alength / distance;
  dy = y2 + (y1 - y2) * alength / distance;
  k = awidth / alength;
  x2o = x2 - dx;
  y2o = dy - y2;
  x3 = y2o * k + dx;
  y3 = x2o * k + dy;
  x4 = dx - y2o * k;
  y4 = dy - x2o * k;
  tft.drawLine(x1, y1, x2, y2, colour);
  tft.drawLine(x1, y1, dx, dy, colour);
  tft.drawLine(x3, y3, x4, y4, colour);
  tft.drawLine(x3, y3, x2, y2, colour);
  tft.drawLine(x2, y2, x4, y4, colour);
}

*/

//#####################################################################
void PrintText(int x, int y, String text, int colour, byte text_size) {
  tft.setCursor(x, y);
  tft.setTextColor(colour);
  tft.setTextSize(text_size);
  tft.print(text);
  tft.setTextColor(YELLOW); // Default colour
  tft.setTextSize(2);       // Default Text Size
}
//#####################################################################
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available()) gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

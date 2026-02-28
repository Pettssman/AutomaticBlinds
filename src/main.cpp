#include <Arduino.h>
#include <AccelStepper.h>
#include <time.h>
#include "cred.h"
#include <BlynkSimpleEsp32.h>

// Constants
// Motor pins
#define IN1 32
#define IN2 25
#define IN3 27
#define IN4 12

// Motor settings
const int stepsPerRevolution = 4096 / 2;
const int stepsToOpenAndClose = 70260; // 70260 steps for a full open/close cycle

// Timing constants
const unsigned long BLINDS_TIMEOUT = 10 * 60 * 1000; // 10 minutes
const double GRADUAL_OPENING_PERCENTAGE = 0.05;
const int TIME_BEFORE_GRADUAL_OPENING = 5;

// NTP settings
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;     // GMT offset for CET (1 hour)
const int daylightOffset_sec = 3600; // Daylight saving time (1 hour)

// Control flags
bool ner = false;
bool upp = false;
bool open_full = false;
bool close_full = false;
bool emergency_stop = false;
bool blindsOpenedToday = false;
bool blindsClosedToday = false;
bool incrementalOpeningStarted = false;

// Time settings
int openHour = 7;
int openMinute = 0;
int closeHour = 20;
int closeMinute = 0;

// Create instances
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);
BlynkTimer timer;

// Function declarations
void checkTime();
void openBlinds();
void closeBlinds();
void openBlindsGradually();
void RunAll();
bool hasTimedOut(unsigned long startTime, unsigned long timeoutDuration);

// Blynk event handlers
BLYNK_WRITE(V0)
{
  ner = param.asInt();
}

BLYNK_WRITE(V1)
{
  upp = param.asInt();
}

BLYNK_WRITE(V4)
{
  open_full = param.asInt();
}

BLYNK_WRITE(V5)
{
  close_full = param.asInt();
}

BLYNK_WRITE(V6)
{
  emergency_stop = param.asInt();
  if (emergency_stop)
  {
    stepper.stop();
    stepper.setSpeed(0);
    stepper.setCurrentPosition(stepper.currentPosition());
    Serial.println("Emergency stop activated!");
  }
}

BLYNK_WRITE(V3)
{
  TimeInputParam t(param);
  if (t.hasStartTime())
  {
    openHour = t.getStartHour();
    openMinute = t.getStartMinute();
    blindsOpenedToday = false;
    Serial.printf("New open time: %d:%d\n", openHour, openMinute);
  }
}

BLYNK_WRITE(V2)
{
  TimeInputParam t(param);
  if (t.hasStartTime())
  {
    closeHour = t.getStartHour();
    closeMinute = t.getStartMinute();
    blindsClosedToday = false;
    Serial.printf("New close time: %d:%d\n", closeHour, closeMinute);
  }
}

bool hasTimedOut(unsigned long startTime, unsigned long timeoutDuration)
{
  if (millis() - startTime > timeoutDuration)
  {
    Serial.println("Operation timed out.");
    return true;
  }
  return false;
}

void RunAll()
{
  Blynk.run();
  timer.run();
  stepper.run();
}

void openBlinds()
{
  Serial.println("Opening blinds automatically...");
  unsigned long startTime = millis();
  stepper.moveTo(-stepsToOpenAndClose);

  while (stepper.distanceToGo() != 0 && !emergency_stop)
  {
    RunAll();
    if (hasTimedOut(startTime, BLINDS_TIMEOUT))
    {
      Serial.println("Stopping motor due to timeout.");
      break;
    }
  }

  if (emergency_stop)
  {
    Serial.println("Emergency stop triggered during opening.");
    emergency_stop = false;
  }

  stepper.setSpeed(0);
}

void closeBlinds()
{
  Serial.println("Closing blinds automatically...");
  unsigned long startTime = millis();
  stepper.moveTo(0);

  while (stepper.distanceToGo() != 0)
  {
    RunAll();
    if (hasTimedOut(startTime, BLINDS_TIMEOUT))
    {
      Serial.println("Stopping motor due to timeout.");
      break;
    }
  }
  stepper.setSpeed(0);
}

void openBlindsGradually()
{
  Serial.println("Opening blinds incrementally...");
  unsigned long startTime = millis();
  float originalSpeed = stepper.maxSpeed();

  if (stepper.currentPosition() < -stepsToOpenAndClose * GRADUAL_OPENING_PERCENTAGE)
    return;

  stepper.moveTo(-stepsToOpenAndClose * GRADUAL_OPENING_PERCENTAGE);

  while (stepper.distanceToGo() != 0)
  {
    RunAll();
    if (hasTimedOut(startTime, BLINDS_TIMEOUT))
    {
      Serial.println("Stopping motor due to timeout during incremental opening.");
      break;
    }
  }
  stepper.setSpeed(0);
  stepper.setMaxSpeed(originalSpeed);
}

void checkTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }

  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;

  Serial.printf("Time %02d:%02d\n", currentHour, currentMinute);
  Serial.printf("Blinds opened: %s, closed: %s\n",
                blindsOpenedToday ? "true" : "false",
                blindsClosedToday ? "true" : "false");
  Serial.println(stepper.currentPosition());

  // Calculate gradual opening time
  int gradualStartHour = openHour;
  int gradualStartMinute = openMinute - TIME_BEFORE_GRADUAL_OPENING;

  if (gradualStartMinute < 0)
  {
    gradualStartMinute += 60;
    gradualStartHour -= 1;
    if (gradualStartHour < 0)
    {
      gradualStartHour = 23;
    }
  }

  Serial.printf("Gradual start: %02d:%02d\n", gradualStartHour, gradualStartMinute);

  // Handle gradual opening
  if (!blindsOpenedToday && !incrementalOpeningStarted &&
      (currentHour == gradualStartHour && currentMinute == gradualStartMinute))
  {
    incrementalOpeningStarted = true;
    openBlindsGradually();
  }

  // Handle full opening
  if (currentHour == openHour && currentMinute == openMinute && !blindsOpenedToday)
  {
    blindsOpenedToday = true;
    openBlinds();
    blindsClosedToday = false;
    incrementalOpeningStarted = false;
  }

  // Handle closing
  if (currentHour == closeHour && currentMinute == closeMinute && !blindsClosedToday)
  {
    blindsClosedToday = true;
    closeBlinds();
    blindsOpenedToday = false;
  }

  // Reset flags at midnight
  if (currentHour == 0 && currentMinute == 0)
  {
    blindsOpenedToday = false;
    blindsClosedToday = false;
    incrementalOpeningStarted = false;
  }
  Serial.println("");
}

void setup()
{
  Serial.begin(115200);

  stepper.setMaxSpeed(300.0);
  stepper.setAcceleration(50.0);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  timer.setInterval(10000L, checkTime);

  timer.setInterval(1L * 60L * 60L * 1000L, []()
                    {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("NTP re-sync triggered."); });
}

void loop()
{
  RunAll();

  if (open_full)
    openBlinds();
  if (close_full)
    closeBlinds();

  if (ner || upp)
  {
    long dir = ner ? 1e6 : -1e6;
    stepper.moveTo(stepper.currentPosition() + dir);
    Serial.print("Manual move ");
    Serial.println(upp ? "up" : "down");

    while (ner || upp)
    {
      RunAll();
    }

    stepper.moveTo(stepper.currentPosition());
    stepper.setCurrentPosition(stepper.currentPosition());
  }
}

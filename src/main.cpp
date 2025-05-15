#include <Arduino.h>
#include <AccelStepper.h>
#include <time.h>

#include "cred.h"

#include <BlynkSimpleEsp32.h>

bool ner;
bool upp;
bool open_full;
bool close_full;
bool emergency_stop = false; // Emergency stop flag
// Flags to prevent multiple triggers
bool blindsOpenedToday = false;
bool blindsClosedToday = false;
bool incrementalOpeningStarted = false; // Add a new flag to track incremental opening
int openHour = 7;
int openMinute = 0;
int closeHour = 20;
int closeMinute = 0;
// Motor pins (ULN2003 for 28BYJ-48 stepper)
#define IN1 32
#define IN2 25
#define IN3 27
#define IN4 12

const int stepsPerRevolution = 4096 / 2;
const int stepsToOpenAndClose = 90845;

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

BlynkTimer timer;

// NTP server settings
const char *ntpServer = "pool.ntp.org"; // NTP server
const long gmtOffset_sec = 3600;        // GMT offset for CET (1 hour = 3600 seconds)
const int daylightOffset_sec = 3600;    // Additional offset for daylight saving time (1 hour = 3600 seconds)

void checkTime(); // <<<< MAKE SURE THIS IS HERE!!!

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

BLYNK_WRITE(V6) // Emergency stop button
{
  emergency_stop = param.asInt();
  if (emergency_stop)
  {
    // Immediately stop the motor
    stepper.stop();
    stepper.setSpeed(0);
    stepper.setCurrentPosition(stepper.currentPosition());
    Serial.println("Emergency stop activated!");
  }
}

BLYNK_WRITE(V3) // Open Time from app
{
  TimeInputParam t(param);

  if (t.hasStartTime())
  {
    openHour = t.getStartHour();
    openMinute = t.getStartMinute();
    blindsOpenedToday = false;
    Serial.print("New open time: ");
    Serial.print(openHour);
    Serial.print(":");
    Serial.println(openMinute);
  }
}

BLYNK_WRITE(V2) // Close Time from app
{
  TimeInputParam t(param);

  // Timeout duration in milliseconds
}

// Timeout duration in milliseconds
const unsigned long BLINDS_TIMEOUT = 10 * 60 * 1000; // 10 minutes

// Gradual opening percentage
const double GRADUAL_OPENING_PERCENTAGE = 0.05; // 1%
// Time before gradual opening in minutes
const int TIME_BEFORE_GRADUAL_OPENING = 5;

// --- Timeout function ---
bool hasTimedOut(unsigned long startTime, unsigned long timeoutDuration)
{
  if (millis() - startTime > timeoutDuration)
  {
    Serial.println("Operation timed out.");
    return true;
  }
  return false;
}

// --- Run all functions ---
// This function is called in the loop to run all tasks
void RunAll()
{
  Blynk.run();
  timer.run();
  stepper.run();
}

// --- Blinds control functions ---
void openBlinds()
{
  Serial.println("Opening blinds automatically...");
  unsigned long startTime = millis();
  stepper.moveTo(-stepsToOpenAndClose);

  while (stepper.distanceToGo() != 0 && !emergency_stop)
  {
    RunAll(); // Run motor until it reaches the target position or stop button pressed
    if (hasTimedOut(startTime, BLINDS_TIMEOUT))
    {
      Serial.println("Stopping motor due to timeout.");
      break;
    }
  }

  if (emergency_stop)
  {
    Serial.println("Emergency stop triggered during opening.");
    emergency_stop = false; // Reset the flag
  }

  stepper.setSpeed(0);
}

void closeBlinds()
{
  Serial.println("Closing blinds automatically...");
  unsigned long startTime = millis();
  stepper.moveTo(0); // Initialize with blinds closed

  while (stepper.distanceToGo() != 0)
  {
    RunAll(); // Run motor until it reaches the target position
    if (hasTimedOut(startTime, BLINDS_TIMEOUT))
    {
      Serial.println("Stopping motor due to timeout.");
      break;
    }
  }
  stepper.setSpeed(0);
}

// Helper function to open blinds incrementally
void openBlindsGradually()
{
  Serial.println("Opening blinds incrementally...");
  unsigned long startTime = millis();

  // Save current speed setting
  float originalSpeed = stepper.maxSpeed();

  // Set lower speed for gradual opening
  // stepper.setMaxSpeed(10.0);

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
  // Restore original speed setting
  stepper.setMaxSpeed(originalSpeed);
}

// --- Check Time function ---
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
  Serial.print("Blinds opened today: ");
  Serial.println(blindsOpenedToday ? "true" : "false");
  Serial.print("Blinds closed today: ");
  Serial.println(blindsClosedToday ? "true" : "false");
  Serial.println(stepper.currentPosition());

  // Gradual opening logic
  int gradualStartHour = openHour;
  int gradualStartMinute = openMinute - TIME_BEFORE_GRADUAL_OPENING;

  // Adjust for negative minutes
  if (gradualStartMinute < 0)
  {
    gradualStartMinute += 60;
    gradualStartHour -= 1;
    if (gradualStartHour < 0)
    {
      gradualStartHour = 23; // Wrap around to the previous day
    }
  }

  Serial.printf("Gradual start: %02d:%02d\n", gradualStartHour, gradualStartMinute);

  if (!blindsOpenedToday && !incrementalOpeningStarted &&
      (currentHour == gradualStartHour && currentMinute == gradualStartMinute))
  {
    incrementalOpeningStarted = true; // Set the flag to prevent multiple triggers
    openBlindsGradually();
  }

  // Full opening at the scheduled time
  if (currentHour == openHour && currentMinute == openMinute && !blindsOpenedToday)
  {
    blindsOpenedToday = true;
    openBlinds();
    blindsClosedToday = false;         // Reset close flag
    incrementalOpeningStarted = false; // Reset incremental flag for the next day
  }

  // Closing logic
  if (currentHour == closeHour && currentMinute == closeMinute && !blindsClosedToday)
  {
    blindsClosedToday = true;
    closeBlinds();
    blindsOpenedToday = false; // Reset open flag
  }

  // Reset flags at midnight
  if (currentHour == 0 && currentMinute == 0)
  {
    blindsOpenedToday = false;
    blindsClosedToday = false;
    incrementalOpeningStarted = false; // Reset incremental flag for the new day
  }
  Serial.println("");
}

void setup()
{
  Serial.begin(115200);

  // Stepper motor settings
  stepper.setMaxSpeed(150.0);    // steps per second
  stepper.setAcceleration(50.0); // steps per second^2

  // Connect to WiFi and Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Set up timer to check every 10 seconds
  timer.setInterval(10000L, checkTime);

  // Re-sync NTP every hour (in milliseconds)
  timer.setInterval(1L * 60L * 60L * 1000L, []()
                    {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("NTP re-sync triggered."); });
}

void loop()
{
  RunAll();

  // Check if the blinds should be opened or closed based on the time
  if (open_full)
    openBlinds();
  if (close_full)
    closeBlinds();

  if (ner || upp)
  {
    long dir = ner ? 1e6 : -1e6;
    stepper.moveTo(stepper.currentPosition() + dir);

    while (ner || upp)
    {
      RunAll();
    }

    stepper.moveTo(stepper.currentPosition());
    stepper.setCurrentPosition(stepper.currentPosition());
  }
}

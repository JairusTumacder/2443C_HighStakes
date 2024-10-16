/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// Drivetrain           drivetrain    1, 2, 4, 5, 7
// Motor3               motor         3
// Motor6               motor         6
// intake               motor         8
// arm                  motor         9
// Rotation10           rotation      10
// mogoMech             digital_out   A
// ---- END VEXCODE CONFIGURED DEVICES ----

// Goals (Before Comp): Create 2 Autons (2 for red and flipped on the other side
// of the field) and a 46+ Skills Score P.S. All using just PID movements
// hopefully :) Goals (After Comp): Make an Autonomous Selector on the brain
// screen and refine the regular and skills autos from the previous comp before
// our next comp P.S. Maybe also figure out how to use odometry with PID >:(

#include "vex.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  vexcodeInit();
} // Do not run commented code pls :) P.S. the commented code is untested so I
  // do not know if it works or not but do not try it without me

const double wheelRadius = 1.625;

bool enableDrivePID = true;
bool resetDriveSensor = false;

double kP = 0;
double kI = 0;
double kD = 0;

double yEnc = 0;
double prevYEnc = 0;

double desiredY = 0;

double error = 0;
double prevError = 0;
double derivative = 0;
double totalError = 0;

double turningKP = 0;
double turningKI = 0;
double turningKD = 0;

double absoluteTheta = 0;

double desiredTheta = 0;

double turningError = 0;
double prevTurningError = 0;
double turningDerivative = 0;
double totalTurningError = 0;

int chassisControl() {
  while (enableDrivePID) {
    if (resetDriveSensor) {
      resetDriveSensor = false;
      rightMotorB.resetPosition();
    }

    yEnc = rightMotorB.position(rotationUnits::deg);

    error = (2 * M_PI * (yEnc - prevYEnc) * wheelRadius) - desiredY;

    derivative = error - prevError;

    totalError += error;

    double output = error * kP + totalError * kI + derivative * kD;

    absoluteTheta = 0 - DrivetrainInertial.angle(deg);

    turningError = absoluteTheta - desiredTheta;

    turningDerivative = turningError - prevTurningError;

    totalTurningError += turningError;

    double turningOutput = turningError * turningKP +
                           totalTurningError * turningKI +
                           turningDerivative * turningKD;

    LeftDriveSmart.spin(fwd, output + turningOutput, voltageUnits::volt);
    RightDriveSmart.spin(fwd, output - turningOutput, voltageUnits::volt);

    prevYEnc = yEnc;
    prevError = error;
    prevTurningError = turningError;
    vex::task::sleep(20);
  }
  return 1;
}

void redAuto1() {}

void redAuto2() {}

void blueAuto1() {}

void blueAuto2() {}

void skills() {}

void autonomous(void) {
  vex::task chassisPID(chassisControl);

  resetDriveSensor = true;
  desiredY = 24;
  desiredTheta = 90;

  // redAuto1();
  // redAuto2();
  // blueAuto1();
  // blueAuto2();
  // skills();
}

// Toggles the mogo mech; changing between its on and off positions with the
// press of a button
void toggleMogoMech() {
  if (!mogoMech.value()) {
    mogoMech.set(true);
  } else {
    mogoMech.set(false);
  }
}

int intakeStates = 0;

// Toggles the intake and conveyor; changing between intaking, outtaking, and
// stopping states with a press of a button
// void toggleIntake() {
//   intake.setVelocity(100, pct);
//   if (intakeStates < 2) {
//     intakeStates++;
//   } else {
//     intakeStates = 0;
//   }

//   if (intakeStates == 1) {
//     intake.spin(fwd);
//     runningIntake = true;
//     while(runningIntake){
//     if(intake.power() == 0){
//       intake.spin(reverse);
//       wait(0.5, msec);
//       intake.stop();
//       intakeStates = 0;
//       runningIntake = false;
//     }
//     }
//   } else if (intakeStates == 2) {
//     intake.spin(reverse);
//   } else {
//     intake.stop();
//   }
// }

int armStates = 0;
bool isStaged = false;

// Toggles the arm; changing between staging and scoring position
// with the press of a button
void armToggle() {
  if (armStates < 1) {
    armStates++;
  } else {
    armStates = 0;
  }

  if (armStates == 1) {
    // Arm's Staging Position
    arm.spin(fwd);
    waitUntil(Rotation10.position(rotationUnits::deg) <= 0);
    arm.stop();
    isStaged = true;
  } else {
    // Arm's Scoring Position
    arm.spin(fwd);
    waitUntil(Rotation10.position(rotationUnits::deg) <= 0);
    arm.stop();
    isStaged = false;
  }
}

// Reset the arm's position to its starting position with the press of a button
void resetArm() {
  arm.spin(directionType::rev);
  waitUntil(Rotation10.position(rotationUnits::deg) <= 0);
  arm.stop();
}

// Prints the important values needed to test the driver controls
void driverDashboard() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 3);
  Controller1.Screen.print(Rotation10.position(degrees));
  Controller1.Screen.setCursor(2, 3);
  Controller1.Screen.print(intake.power());
}

void usercontrol(void) {
  while (1) {
    //Sets the intake speed to max
    intake.setVelocity(127, pct);

    //Sets the arm to hold which stops the motor from moving by "holding" it there
    arm.setStopping(brakeType::hold);
    Controller1.ButtonL1.pressed(toggleMogoMech);

    if (Controller1.ButtonR1.pressing()) {
      intake.spin(fwd);
      while (isStaged) {
        if (intake.power() == 0) {
          intake.spin(directionType::rev);
          wait(0.5, msec);
          intake.stop();
          break;
        }
      }
    } else if (Controller1.ButtonR2.pressing()) {
      intake.spin(directionType::rev);
    }

    if (Controller1.ButtonL2.pressing()) {
      Controller1.ButtonR1.pressed(armToggle);
      Controller1.ButtonR2.pressed(resetArm);
    }

    driverDashboard();
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}

using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern motor leftMotorA;
extern motor leftMotorB;
extern motor leftMotorC;
extern motor_group LeftDriveSmart;
extern motor rightMotorA;
extern motor rightMotorB;
extern motor rightMotorC;
extern motor_group RightDriveSmart;
extern motor intake;
extern motor arm;
extern rotation Rotation10;
extern inertial DrivetrainInertial;
extern digital_out mogoMech;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
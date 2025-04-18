#include "main.h"
#include "lemlib/api.hpp"

//----------------------------------------------------------------------------------Device Setup----------------------------------------------------------------------------------
//--START Device Setup--

pros::MotorGroup Left ({-20, 13, -7}, pros::MotorGears::blue);
pros::MotorGroup Right ({14, 18, -17}, pros::MotorGears::blue);
pros::Motor Arm (8, pros::MotorGears::green);
pros::Motor IntakeFlex (-2, pros::MotorGears::green);
pros::Motor IntakeHook (-1, pros::MotorGears::blue);

pros::Controller ParaRAID(pros::E_CONTROLLER_MASTER);


pros::adi::DigitalOut Lift('B');
pros::adi::DigitalOut Eject('H'); //doesn't exist
pros::adi::DigitalOut MobileGoal('E');
pros::adi::DigitalOut DoinkerLeft('A');
pros::adi::DigitalOut DoinkerRight('D');

pros::Rotation LadyBrownOdom(-3);
pros::Optical VSensor(4);
pros::Imu Inertial(10); 
pros::Rotation HorizontalEnc(9);
pros::Rotation VerticalEnc(-12);

lemlib::TrackingWheel Horizontal(&HorizontalEnc, lemlib::Omniwheel::NEW_2, 0.0); //change
lemlib::TrackingWheel Vertical(&VerticalEnc, lemlib::Omniwheel::NEW_2, -0.1);

// drivetrain settings
lemlib::Drivetrain drivetrain(&Left, &Right, 12.8, lemlib::Omniwheel::NEW_325, 450, 2); //left right track width, wheel type, rpm, drift

// Forward PID
lemlib::ControllerSettings linearController(6,  // proportional gain (kP)
                                            0, // integral gain (kI)
                                            12, // derivative gain (kD)
                                            1, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);
// Turn PID
lemlib::ControllerSettings angularController(4.0, // proportional gain (kP)
                                             0.0, // integral gain (kI)
                                             35, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             50, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             300, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors setup


lemlib::OdomSensors sensors(&Vertical, nullptr, &Horizontal, nullptr, &Inertial);

//

lemlib::ExpoDriveCurve throttleCurve(2, // joystick deadband out of 127
                                     2.5, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain og 1.019
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(2, // joystick deadband out of 127
                                  2.5, // minimum output where drivetrain will move out of 127
                                  1.0 // expo curve gain og 1.019
);


// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//--END Device Setup-- 



//----------------------------------------------------------------------------------Global Variables----------------------------------------------------------------------------------


bool TeamColor = true; //true = blue, red = false 
int TeamColorInt = 1;
bool skillz = false;

double ArmPos() {
    double armPos = LadyBrownOdom.get_position()/100.000;
    return armPos;
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    LadyBrownOdom.reset_position();

    if (TeamColor) {
        TeamColorInt = 1;
    } else {
        TeamColorInt = -1;
    }

    
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "------------X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "------------Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "------------Theta: %f", chassis.getPose().theta); // heading
            
            pros::lcd::print(5, "--------Arm Position: %f", ArmPos());
            // delay to save resources
            pros::delay(50);
        }
    });
}

void disabled() {  //well disable on bot enable

 
}


void competition_initialize() { //Auto Selector


}



//----------------------------------------------------------------------------------Auto Setup----------------------------------------------------------------------------------

bool DontEject = false;
bool EjectStay = false;



//--------------------------------------------------------------------------------Doinker--------------------------------------------------------------------------------

void DoinkerControl() {
    int DoinkerLeftToggle = 1;
    int DoinkerRightToggle = 1;
    while (true) {
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (DoinkerLeftToggle == 1) {
                DoinkerLeft.set_value(4096);
            } else {
                DoinkerLeft.set_value(0);
            }
            DoinkerLeftToggle *= -1;
            pros::delay(500);
        }
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (DoinkerRightToggle == 1) {
                DoinkerRight.set_value(0);
            } else {
                DoinkerRight.set_value(4096);
            }
            DoinkerRightToggle *= -1;
            pros::delay(500);
        }
        pros::delay(10);
    }
}


//--------------------------------------------------------------------------------Chassis--------------------------------------------------------------------------------

void ChassisControl() {
    while (true) {
        int leftY = ParaRAID.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = ParaRAID.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY*1.1, rightX);

        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -9, 800, {.minSpeed = 127});
            chassis.waitUntilDone();
        }

        pros::delay(20); // Run for 20 ms then update
    }
}

//--------------------------------------------------------------------------------Arm--------------------------------------------------------------------------------


void ArmPIDtoPosition(double target, double timeout) {
    double ArmKp = 4.00; // Proportional Modifier
    double ArmKd = 3.20; // Derivative Modifier
    double error;
    double prevError = 0;
    double derivative = 0;
    double repeated = 0;

    while ((ArmPos() > (target + 0.5)) || (ArmPos() < (target - 0.5))) {
        error = target - ArmPos();
        derivative = error - prevError;

        pros::lcd::print(6, "-----------Arm Error: %f", error);

        Arm.move_velocity((ArmKp * error) + (ArmKd * derivative)); // PID Control


        prevError = error;
        
        pros::delay(20);

        if ((ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) || ((repeated > (timeout/20)) && (timeout != 0))) {
            goto exit;
        }

        repeated += 1;

    }
    exit:
    Arm.brake();
    repeated = 0;
}

double ArmLoadPos = 27.50;
double ArmTipPos = 180.00;
double ScoreAlliancePos = 186.00;

void ArmControl() {
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
    while (true) { 
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            ArmPIDtoPosition(ScoreAlliancePos, 0);
            
        } else if ((ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (ArmPos() < (ArmLoadPos-10.0))) || ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            ArmPIDtoPosition(ArmLoadPos, 0);

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            ArmPIDtoPosition(3.00, 0);
            
        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            Arm.move_velocity(200);

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            Arm.move_velocity(-200);
            if (ArmPos() < 0) {
                LadyBrownOdom.reset_position();
            } 
        } else {
            Arm.brake();
        }
        pros::delay(10);
    }
}


//--------------------------------------------------------------------------------Intake--------------------------------------------------------------------------------


void FuncIntake() {
    int IntakeToggle = 1;
    int IntakeSpeed = 100; //100 for skills, 90 regular
    while (true) {
        if (IntakeHook.get_actual_velocity() == 0 && IntakeToggle == -1) {
            IntakeHook.move_velocity(IntakeSpeed * -2);
        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            while (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                IntakeFlex.move_velocity(IntakeSpeed * -2);
                IntakeHook.move_velocity(IntakeSpeed * -6);
            }
            if (IntakeToggle == 1) {
                IntakeFlex.brake();
                IntakeHook.brake();
            } else {
                IntakeFlex.move_velocity(IntakeSpeed * 2);
                IntakeHook.move_velocity(IntakeSpeed * 6);
            }

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (IntakeToggle == 1) {
                IntakeFlex.move_velocity(IntakeSpeed * 2);
                IntakeHook.move_velocity(IntakeSpeed * 6);
            } else {
                IntakeFlex.brake();
                IntakeHook.brake();
                IntakeHook.move_relative(-100, 600);
                pros::delay(50);
            }
            IntakeToggle *= -1;
            pros::delay(500);
        } else if (IntakeToggle == -1) {
            IntakeFlex.move_velocity(IntakeSpeed * 2);
            IntakeHook.move_velocity(IntakeSpeed * 6);
            pros::delay(10);
        }
        pros::delay(20);
    }
}

void FuncIntakeLift() {
    int IntakeLiftToggle = 1;
    while (true) {
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && !skillz) {
            if (IntakeLiftToggle == 1) {
                Lift.set_value(4096);
                pros::delay(500);
            } else {
                Lift.set_value(0);
                pros::delay(500);
            }
            IntakeLiftToggle *= -1;
            pros::delay(500);
        }

        pros::delay(20);
    }
}

//--------------------------------------------------------------------------------Mogoal--------------------------------------------------------------------------------

int MogoToggle = 1;
void FuncMogo() {
    while (true) {
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (MogoToggle == 1) {
                MobileGoal.set_value(4095);
                ParaRAID.set_text(0, 0, "On ");
            } else {
                MobileGoal.set_value(0);
                ParaRAID.set_text(0, 0, "Off");
            }

            MogoToggle *= -1;
            pros::delay(500);
        }
        pros::delay(5);
    }
}

//--------------------------------------------------------------------------------Eject--------------------------------------------------------------------------------

void DriverEject() {
    while (true) {
        int opticalHue = VSensor.get_hue();
        pros::lcd::print(3, "------------Optical Hue: %d", opticalHue);
        //red = 30-50
        //blue = 100-150

        if (!TeamColor && (opticalHue > 100) && (MogoToggle == -1)) {
            pros::delay(100);
            IntakeHook.brake();
            pros::delay(600);
            IntakeHook.move_velocity(600);
        } else if ((opticalHue < 50) && (MogoToggle == -1)) {
            pros::delay(300);
            IntakeHook.brake();
            pros::delay(600);
            IntakeHook.move_velocity(600);
        }
        pros::delay(10);
    }
}
//----------------------------------------------------------------------------------Auto Routes----------------------------------------------------------------------------------


void Negative_A0() {
    chassis.setPose(12, 0, 180);
    chassis.moveToPoint(24, 24, 1000, {.forwards = false});

    chassis.waitUntilDone();
}

void Positive_GoalRush_5R() {
    chassis.setPose(47 * TeamColorInt, 7.5, 0);
    pros::delay(20);
    
    IntakeFlex.move_velocity(200);
    if (TeamColor) {
        DoinkerLeft.set_value(4096);
    } else {
        DoinkerRight.set_value(4096);
    }

    chassis.moveToPoint(47 * TeamColorInt, 26, 1000, {.minSpeed = 127});
    chassis.moveToPoint(24 * TeamColorInt, 24, 1000, {.forwards = false});
    chassis.waitUntilDone();

    MobileGoal.set_value(4096);
    MogoToggle = -1; // because we are holding the a Mogoal

    pros::delay(500);
    IntakeHook.move_velocity(600);

    //------------------------------------------------

    chassis.turnToHeading(175 * TeamColorInt, 1000);
    chassis.waitUntilDone();

    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);

    chassis.turnToHeading(150 * TeamColorInt, 1000);
    chassis.moveToPoint(36 * TeamColorInt, 12, 1000, {.forwards = false});
    chassis.turnToPoint(76 * TeamColorInt, -24, 1000);
    chassis.moveToPoint(76 * TeamColorInt, -24, 1000, {.minSpeed = 127});

    //----------------------------------------------------

    // chassis.waitUntilDone();
    // pros::delay(1500);

    // chassis.moveToPoint(60, 0, 500, {.forwards = false, .maxSpeed = 80});
    // chassis.waitUntilDone();

    // Lift.set_value(4095);
    // pros::delay(500);

    // chassis.moveToPoint(65, -24, 2000, {.minSpeed = 80});
    // pros::delay(1000);
    
    // IntakeFlex.move_velocity(-200);
    
    // chassis.moveToPoint(48, 24, 1000, {.forwards = false});

    chassis.waitUntilDone();
}

void Negative_A1_5R() {
    chassis.setPose(12 * TeamColorInt, -12, -120);
    LadyBrownOdom.set_position(ArmLoadPos * 100);
    pros::delay(20);

    ArmPIDtoPosition(ScoreAlliancePos, 500);
    chassis.moveToPoint(24 * TeamColorInt, 24, 1600, {.forwards = false, .minSpeed = 30});
    pros::delay(500);
    ArmPIDtoPosition(3.00, 650);

    chassis.waitUntilDone();

    //-----------------------------------------------


    MobileGoal.set_value(4095);
    MogoToggle = -1; // because we are holding the a Mogoal

    pros::delay(250);
    chassis.turnToHeading(45 * TeamColorInt, 800);
    chassis.waitUntilDone();

    IntakeFlex.move_velocity(200);

    chassis.moveToPoint(36 * TeamColorInt, 32, 1500);
    chassis.waitUntilDone();
    IntakeHook.move_velocity(600);

    chassis.swingToHeading(85 * TeamColorInt, DriveSide::RIGHT, 700);
    chassis.moveToPoint(40 * TeamColorInt, 32, 1000, {.minSpeed = 70});

    chassis.swingToHeading(160 * TeamColorInt, DriveSide::RIGHT, 1000);
    chassis.turnToHeading(-170 * TeamColorInt, 1000);

    chassis.moveToPoint(46 * TeamColorInt, 24, 1200, {.minSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToPoint(62.5 * TeamColorInt, -24, 1000);
    chassis.moveToPoint(62.5 * TeamColorInt, -24, 1600, {.minSpeed = 127});

    chassis.moveToPoint(60, -5, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();

    // Lift.set_value(4095);
    // pros::delay(500);

    // chassis.moveToPoint(65, -24, 2000, {.minSpeed = 80});
    // pros::delay(1000);
    
    // IntakeFlex.move_velocity(-200);
    
    // chassis.moveToPoint(48, 24, 1000, {.forwards = false});

    chassis.waitUntilDone();
    
}


void Negative_A1_5R_PC() {
    Negative_A1_5R();

    //-----------------------------------------------

    chassis.turnToPoint(0, 0, 1000);
    chassis.waitUntilDone();
    IntakeHook.brake();
    IntakeFlex.brake();
    chassis.moveToPoint(-57, -2, 2000);
    pros::delay(800);
    DoinkerLeft.set_value(4096);
    chassis.turnToHeading(-45, 1000);
    DoinkerLeft.set_value(0);
}

void Negative_A1_5R_TB() {
    Negative_A1_5R();

    //-----------------------------------------------

    // chassis.turnToPoint(12, 60, 1000);
    // chassis.waitUntilDone();
    // IntakeHook.brake();
    // IntakeFlex.brake();
    // chassis.moveToPoint(12, 60, 1000);


    //-if time persists, grab center ring for 6 rings-
}

//----------------------------------------------------------------------------------Auto----------------------------------------------------------------------------------

void autonomous() {
    Negative_A1_5R();

}

//----------------------------------------------------------------------------------opcontrol----------------------------------------------------------------------------------


void opcontrol() { //Driver
    pros::rtos::Task TaskChassisControl(ChassisControl);
    pros::rtos::Task TaskArmControl(ArmControl);
    pros::rtos::Task TaskFuncIntake(FuncIntake);
    pros::rtos::Task TaskFuncMogo(FuncMogo);
    pros::rtos::Task TaskEject(DriverEject);
    pros::rtos::Task TaskDoiner(DoinkerControl);
    //pros::rtos::Task TaskFuncIntakeLift(FuncIntakeLift);

    
    while (true) {
        

		pros::delay(20);                               // Run for 20 ms then update
    }
}
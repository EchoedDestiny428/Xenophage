#include "main.h"
#include "lemlib/api.hpp"

//----------------------------------------------------------------------------------Device Setup----------------------------------------------------------------------------------
//--START Device Setup--

pros::MotorGroup Left ({11, -16, -7}, pros::MotorGears::blue);
pros::MotorGroup Right ({12, 15, -14}, pros::MotorGears::blue);
pros::Motor Arm (5, pros::MotorGears::green);
pros::Motor IntakeFlex (-2, pros::MotorGears::green);
pros::Motor IntakeHook (-1, pros::MotorGears::blue);

pros::Controller ParaRAID(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalOut Lift('B');
pros::adi::DigitalOut MobileGoal('C');
pros::adi::DigitalOut DoinkerLeft('E');
pros::adi::DigitalOut DoinkerRight('A');

pros::Rotation LadyBrownOdom(-3);
pros::Optical VSensor(8);
pros::Distance Distance(10);
pros::Imu Inertial(21); 
pros::Rotation HorizontalEnc(-6);
pros::Rotation VerticalEnc(13);

lemlib::TrackingWheel Horizontal(&HorizontalEnc, lemlib::Omniwheel::NEW_2, 1.3); //change
lemlib::TrackingWheel Vertical(&VerticalEnc, lemlib::Omniwheel::NEW_2, -1.45);

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
int TeamColorInt;
bool skillz = false;

double ArmPos() {
    double armPos = LadyBrownOdom.get_position()/100.000;
    return armPos;
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    LadyBrownOdom.reset_position();
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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
            
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

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

int redHueMax = 30;
int redHueMin = 0;

int blueHueMax = 240;
int blueHueMin = 160;

// red = 30-50
// blue = 100-150

bool isThereARing() {
    if (Distance.get() < 50) {
        return true;
    } else {
        return false;
    }
}




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
                DoinkerRight.set_value(4096);
            } else {
                DoinkerRight.set_value(0);
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

        chassis.arcade(leftY, rightX*1.1);

        // if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     chassis.setPose(0, 0, 0);
        //     chassis.moveToPoint(0, -9, 800, {.forwards = false, .minSpeed = 50});
        //     chassis.waitUntilDone();
        // }

        pros::delay(20); // Run for 20 ms then update
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

//--------------------------------------------------------------------------------Eject & ColorSensor--------------------------------------------------------------------------------

int IntakeToggle = 1;

void DriverEject() {
    VSensor.set_led_pwm(100);
    IntakeHook.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    float EjectDistance = 260.0;
    bool RingColor; //true = blue, false = red

    while (true) {
        if ((Distance.get() < 50) && (MogoToggle == -1)) {
            IntakeHook.tare_position();
            IntakeHook.move_velocity(300);

            while (IntakeHook.get_position() < EjectDistance) {
                int opticalHue = VSensor.get_hue();

                if (TeamColor && (opticalHue > blueHueMin) && (opticalHue < blueHueMax)) {
                    goto exitEject;
                } else if (!TeamColor && (opticalHue > redHueMin) && (opticalHue < redHueMax)) {
                    goto exitEject;
                } else if (IntakeToggle == 1) {
                    goto intakeActuallyTurnedOffToAnnoyMeAndIHaveToCreateASolution;
                }

                pros::delay(5);
            }
            IntakeHook.brake();
            pros::delay(400);
            

            exitEject:
            IntakeHook.move_velocity(600);
            break;

            intakeActuallyTurnedOffToAnnoyMeAndIHaveToCreateASolution:
            IntakeHook.brake();
        }
        pros::delay(20);
    }
}

bool autonomousRunning = true;

void AutoEject() {
    VSensor.set_led_pwm(100);
    IntakeHook.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    float EjectDistance = 260.0;
    bool RingColor; //true = blue, false = red

    while (autonomousRunning) {
        if ((Distance.get() < 50)) {
            IntakeHook.tare_position();
            IntakeHook.move_velocity(300);

            while (IntakeHook.get_position() < EjectDistance) {
                int opticalHue = VSensor.get_hue();

                if (TeamColor && (opticalHue > blueHueMin) && (opticalHue < blueHueMax)) {
                    goto exitEject;
                } else if (!TeamColor && (opticalHue > redHueMin) && (opticalHue < redHueMax)) {
                    goto exitEject;
                }

                pros::delay(5);
            }
            IntakeHook.brake();
            pros::delay(400);
            

            exitEject:
            IntakeHook.move_velocity(600);
        }
        pros::delay(20);
    }
}

//--------------------------------------------------------------------------------Arm--------------------------------------------------------------------------------

double ArmLoadPos = 29.50;
double ArmTipPos = 180.00;
double ScoreAlliancePos = 186.00;
double DescorePos = 160.00;
bool ArmPIDing = false;

void ArmPIDtoPosition(double target, double timeout) {
    double ArmKp = 3.50; // Proportional Modifier
    double ArmKd = 3.20; // Derivative Modifier
    double error;
    double prevError = 0;
    double derivative = 0;
    double repeated = 0;
    ArmPIDing = true;

    while ((ArmPos() > (target + 0.5)) || (ArmPos() < (target - 0.5))) {
        error = target - ArmPos();
        derivative = error - prevError;


        Arm.move_velocity((ArmKp * error) + (ArmKd * derivative)); // PID Control

        prevError = error;
        
        pros::delay(20);

        if (((ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) || ((repeated >= (timeout/25)) && (timeout != 0))) && repeated > 10) {
            goto exit;
        }

        repeated += 1;

    }
    exit:
    Arm.brake();
    repeated = 0;
    ArmPIDing = false;
}



void ArmControl() {
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
    while (true) { 
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            ArmPIDtoPosition(DescorePos, 0);
            
        } else if ((ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (ArmPos() < (ArmLoadPos-10.0))) || ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            ArmPIDtoPosition(ArmLoadPos, 0);

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            ArmPIDtoPosition(3.00, 0);
            
        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            if (((LadyBrownOdom.get_position()) > ((ArmLoadPos-8)*100)) && ((LadyBrownOdom.get_position()) < ((ArmLoadPos+2)*100))) {
                IntakeHook.move_relative(-120, 600);
                pros::delay(200);
                while (((LadyBrownOdom.get_position()) > ((ArmLoadPos-8)*100)) && ((LadyBrownOdom.get_position()) < ((ArmLoadPos+2)*100))) {
                    Arm.move_velocity(200);
                }
            }
            Arm.move_velocity(200);

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            Arm.move_velocity(-200);
            if (ArmPos() < 0) {
                LadyBrownOdom.reset_position();
            }
        } else if (!ArmPIDing) {
            Arm.brake();
        }
        pros::delay(10);
    }
}




//--------------------------------------------------------------------------------Intake--------------------------------------------------------------------------------


void FuncIntake() {
    int IntakeSpeed = 100; //100 for skills, 90 regular
    while (true) {
        if (((IntakeHook.get_torque() > 0.2) && IntakeHook.get_actual_velocity() < 5 && IntakeToggle == -1)) {
            if (((LadyBrownOdom.get_position()) > ((ArmLoadPos-8)*100)) && ((LadyBrownOdom.get_position()) < ((ArmLoadPos+2)*100))) {
                // IntakeHook.move_relative(-120, 600);
                // pros::delay(200);
                IntakeHook.brake();
                IntakeToggle = 1;
            } else {
                IntakeHook.move_relative(-100, 600);
                pros::delay(150);
                IntakeHook.move_velocity(IntakeSpeed * 6);
            }
        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            while (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                IntakeFlex.move_velocity(IntakeSpeed * -2);
                IntakeHook.move_velocity(IntakeSpeed * -6);
                pros::delay(20);
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
                IntakeHook.move_relative(-30, 600); 
                pros::delay(150);
            }
            IntakeToggle *= -1;
            pros::delay(500);
        } else if (IntakeToggle == -1) {
            // IntakeFlex.move_velocity(IntakeSpeed * 2);
            // IntakeHook.move_velocity(IntakeSpeed * 6);
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

bool doingWallstake = false;


void IntakeJamPrevAuto() {
    while (autonomousRunning) {
        if ((IntakeHook.get_torque() > 0.2) && (IntakeHook.get_actual_velocity() < 1)) {
            if (doingWallstake) {
                IntakeHook.move_velocity(600);
                pros::delay(400);
                IntakeHook.move_relative(-120, 600);
                pros::delay(200);
                IntakeHook.brake();
                goto exitJam;
            } else {
                IntakeHook.move_velocity(-400);
                pros::delay(100);
                IntakeHook.move_velocity(600);
            }
        }

        pros::delay(20);
    }
    exitJam:
}

//----------------------------------------------------------------------------------Macros---------------------------------------------------------------------------------------

void FuncMacros() {
    while (true) {
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            if (((LadyBrownOdom.get_position()) > ((ArmLoadPos-8)*100)) && ((LadyBrownOdom.get_position()) < ((ArmLoadPos+2)*100))) {
                ArmPIDtoPosition(170, 1000);
                ArmPIDtoPosition(ArmLoadPos, 1000);
                IntakeHook.move_velocity(600);
                pros::delay(1000);
                IntakeHook.move_relative(-120, 200);
                pros::delay(200);
                ArmPIDtoPosition(170, 1000);
            }
        }

        pros::delay(50);
    }
}


//----------------------------------------------------------------------------------Auto Routes----------------------------------------------------------------------------------


//----------------------Solo AWP----------------------

void SoloAWP() {
    chassis.setPose(12 * TeamColorInt, -14, -120 * TeamColorInt);
    LadyBrownOdom.set_position(ArmLoadPos * 100);
    Arm.set_zero_position(ArmLoadPos * 10);
    pros::delay(20);

    ArmPIDtoPosition(ScoreAlliancePos + 50, 1000);
    chassis.moveToPoint(26 * TeamColorInt, 24, 1600, {.forwards = false, .minSpeed = 30});
    
    Arm.move_absolute(10, 200);
    chassis.waitUntilDone();

    //-----------------------------------------------

    MobileGoal.set_value(4096);
    MogoToggle = -1; // because we are holding the a Mogoal
    pros::delay(200);

    //-----------------------------------------------

    IntakeFlex.move_velocity(200);
    IntakeHook.move_velocity(600);

    chassis.turnToHeading(100 * TeamColorInt, 800);
    chassis.moveToPoint(42 * TeamColorInt, 22, 1000);
    pros::delay(200);
    chassis.turnToPoint(38 * TeamColorInt, 20, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(42 * TeamColorInt, 18, 1000, {.minSpeed = 100});
    chassis.moveToPoint(-4 * TeamColorInt, 0, 1000, {.maxSpeed = 80});

    chassis.waitUntilDone();
    MobileGoal.set_value(0);
    MogoToggle = 1; // because we droppped the Mogoal

    //-----------------------------------------------
    
    chassis.moveToPoint(-32 * TeamColorInt, 0, 1500, {.maxSpeed = 50});
    chassis.waitUntilDone();
    IntakeHook.brake();


    chassis.turnToHeading(180 * TeamColorInt, 1000);
    chassis.moveToPoint(-32 * TeamColorInt, 24, 1000, {.forwards = false});
    chassis.waitUntilDone();
    IntakeHook.move_velocity(600);

    MobileGoal.set_value(4096);
    MogoToggle = -1; // because we are holding the a Mogoal

    chassis.turnToHeading(-90 * TeamColorInt, 1000);
    //chassis.moveToPoint(-24 * TeamColorInt, 24, 1000);


    chassis.waitUntilDone();
}


//----------------------Positive----------------------

void Positive_GoalRush_3R() {
    pros::rtos::Task PrevJam(IntakeJamPrevAuto);
    //pros::rtos::Task AutoSort(AutoEject);
    chassis.setPose(-33.7 * TeamColorInt, -6, -21 * TeamColorInt);
    if (TeamColor) {
        DoinkerRight.set_value(4096);
    } else {
        DoinkerLeft.set_value(4096);
    }
    IntakeFlex.move_velocity(200);

    chassis.moveToPoint(-46.5 * TeamColorInt, 29.8, 600, {.minSpeed = 127});
    chassis.moveToPoint(-46.5 * TeamColorInt, 29.8, 400);
    pros::delay(300);
    DoinkerRight.set_value(0);
    DoinkerLeft.set_value(0);
    pros::delay(200);

    chassis.moveToPoint(-42.5 * TeamColorInt, 12, 1000, {.forwards = false, .minSpeed = 127});
    pros::delay(300);
    
    if (TeamColor) {
        DoinkerRight.set_value(4096);
    } else {
        DoinkerLeft.set_value(4096);
    }
    

    chassis.turnToHeading(180, 800);
    chassis.turnToPoint(-23 * TeamColorInt, 23, 800, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(150);
    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);

    chassis.moveToPoint(-23 * TeamColorInt, 23, 1000, {.forwards = false}, false);

    //----------------------------------------

    MobileGoal.set_value(4096);
    IntakeHook.move_velocity(600);
    pros::delay(200);

    //----------------------------------------

    chassis.moveToPoint(-54 * TeamColorInt, -6, 400, {.maxSpeed = 50, .minSpeed = 50}, false);
    pros::delay(300);
    MobileGoal.set_value(0);
    IntakeHook.brake();

    chassis.moveToPoint(-54 * TeamColorInt, -6, 800, {});

    chassis.moveToPoint(-65 * TeamColorInt, -17, 1000, {.maxSpeed = 90, .minSpeed = 90});
    // chassis.moveToPoint(-48 * TeamColorInt, -0, 800, {.forwards = false, .maxSpeed = 80, .minSpeed = 80});
    // chassis.moveToPoint(-54 * TeamColorInt, -6, 1000, {.maxSpeed = 10, .minSpeed = 10}, false);
    // IntakeHook.brake();
    // MobileGoal.set_value(0);

    chassis.moveToPoint(-54 * TeamColorInt, -6, 800, {.forwards = false, .maxSpeed = 40, .minSpeed = 30});

    //----------------------------------------

    if (TeamColor) {
        DoinkerLeft.set_value(4096);
    } else {
        DoinkerRight.set_value(4096);
    }

    IntakeHook.brake();
    chassis.moveToPoint(-56 * TeamColorInt, -7, 800);
    chassis.turnToHeading(-50 * TeamColorInt, 800, {}, false);
    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);
    pros::delay(500);

    // if (TeamColor) {
    //     chassis.swingToPoint(-52 * TeamColorInt, 24, DriveSide::RIGHT, 800, {.forwards = false});
    // } else {
    //     chassis.swingToPoint(-52 * TeamColorInt, 24, DriveSide::LEFT, 800, {.forwards = false});
    // }

    chassis.turnToPoint(-44 * TeamColorInt, 12, 800, {.forwards = false});
    chassis.moveToPoint(-44 * TeamColorInt, 12, 800, {.forwards = false}, false);

    chassis.turnToPoint(-40 * TeamColorInt, 35, 800, {.forwards = false});
    chassis.moveToPoint(-40 * TeamColorInt, 35, 800, {.forwards = false}, false);
    MobileGoal.set_value(4096);
    IntakeHook.move_velocity(600);
    pros::delay(400);
    chassis.turnToPoint(-12 * TeamColorInt, 36, 1000);
    chassis.moveToPoint(-12 * TeamColorInt, 36, 1000, {}, false);

    Arm.move_absolute(ArmLoadPos * 10 - 20, 200);

    chassis.waitUntilDone();
}

void Positive_A1_4R() {
    chassis.setPose(-24 * TeamColorInt, -12, 180);
    pros::delay(20);
    chassis.moveToPoint(-24 * TeamColorInt, 24, 1000, {.forwards = false});
    chassis.waitUntilDone();
    MobileGoal.set_value(4096);
    MogoToggle = -1; // because we are holding the a Mogoal
    IntakeHook.move_velocity(600);

    //----------------------------------------

    chassis.turnToHeading(45 * TeamColorInt, 1000);
    chassis.waitUntilDone();
    
    if (TeamColor) {
        DoinkerLeft.set_value(4096);
    } else {
        DoinkerRight.set_value(4096);
    }

    chassis.moveToPoint(-8 * TeamColorInt, 40, 1000);
    chassis.moveToPoint(-24 * TeamColorInt, 24, 1000, {.forwards = false});
    chassis.waitUntilDone();

    chassis.turnToHeading(-65 * TeamColorInt, 400);
    chassis.waitUntilDone();
    IntakeFlex.move_velocity(200);

    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);

    chassis.moveToPoint(-44 * TeamColorInt, 25, 1000, {.minSpeed = 60});

    //----------------------------------------

    chassis.moveToPoint(-56 * TeamColorInt, -3, 1750);
    chassis.turnToHeading(-135 * TeamColorInt, 500);

    chassis.moveToPoint(-72 * TeamColorInt, -24, 900, {.minSpeed = 127});
    chassis.moveToPoint(-53 * TeamColorInt, -3, 800, {.forwards = false, .maxSpeed = 50});

    // perchance stab again


}

//----------------------Negtative----------------------

void Negative_A0() {
    chassis.setPose(12 * TeamColorInt, -12, 180);
    chassis.moveToPoint(24 * TeamColorInt, 24, 1000, {.forwards = false});

    chassis.waitUntilDone();
}

void Negative_RingRush_5R() {
    pros::rtos::Task TaskIntakeJam(IntakeJamPrevAuto);
    //pros::rtos::Task TaskAutoEject(AutoEject);
    Arm.tare_position();
    chassis.setPose(26.5 * TeamColorInt, -2.5, 19 * TeamColorInt);
    pros::delay(20);

    IntakeFlex.move_velocity(200);
    if (TeamColor) {
        DoinkerRight.set_value(4096);
    } else {
        DoinkerLeft.set_value(4096);
    }

    chassis.moveToPoint(39.5 * TeamColorInt, 38.5, 700, {.minSpeed = 127});
    chassis.moveToPoint(39.5 * TeamColorInt, 38.5, 300, {}, false);
    pros::delay(200);

    //----------------------------------------

    if (TeamColor) {
        chassis.swingToPoint(20 * TeamColorInt, 24, DriveSide::LEFT, 500, {.forwards = false});
    } else {
        chassis.swingToPoint(20 * TeamColorInt, 24, DriveSide::RIGHT, 500, {.forwards = false});

    }
    chassis.moveToPoint(20 * TeamColorInt, 24, 800, {.forwards = false, .minSpeed = 30}, false);

    // // while (chassis.getPose().x > 30 * TeamColorInt) {
    // //     int opticalHue = VSensor.get_hue();
    // //     if (isThereARing()) {
    // //         IntakeHook.brake();
    // //     }
    // // }

    MobileGoal.set_value(4096);
    MogoToggle  = -1; // because we are holding the a Mogoal

    IntakeHook.move_velocity(600);
    
    chassis.turnToHeading(70 * TeamColorInt, 400, {}, false);
    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);
    pros::delay(200);
    chassis.turnToHeading(90 * TeamColorInt, 400);

    chassis.moveToPoint(46 * TeamColorInt, 24, 1000, {.maxSpeed = 110}, false);
    pros::delay(300);

    // //----------------------------------------

    chassis.moveToPoint(55 * TeamColorInt, -5, 2000);
    chassis.turnToHeading(131 * TeamColorInt, 500);

    chassis.moveToPoint(70 * TeamColorInt, -16, 900, {.maxSpeed = 100, .minSpeed = 127}, false);
    pros::delay(200);
    chassis.moveToPoint(50 * TeamColorInt, 1, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.moveToPoint(60 * TeamColorInt, -10, 800, {.maxSpeed = 80, .minSpeed = 80});
    chassis.moveToPoint(56 * TeamColorInt, -7, 1000, {.forwards = false, .maxSpeed = 60});

    // //---------------------------------------

    // chassis.waitUntilDone();
}



void Negative_RingRush_5R_ALT() { //untested
    chassis.setPose(47 * TeamColorInt, -1, 0);
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

    chassis.turnToHeading(145 * TeamColorInt, 1000);
    chassis.waitUntilDone();

    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);

    chassis.turnToPoint(58 * TeamColorInt, -10, 1000);
    chassis.moveToPoint(58 * TeamColorInt, -10, 2000);

    chassis.turnToHeading(135 * TeamColorInt, 500);
    chassis.moveToPoint(72 * TeamColorInt, -24, 800, {.minSpeed = 127});
    chassis.moveToPoint(58 * TeamColorInt, -10, 500, {.forwards = false});

    Lift.set_value(4095);
    chassis.moveToPoint(72 * TeamColorInt, -24, 1000, {.minSpeed = 127});
    chassis.moveToPoint(58 * TeamColorInt, -10, 500, {.forwards = false});
    chassis.waitUntilDone();
    Lift.set_value(0);

    //---------------------------------------

    chassis.waitUntilDone();
}

void Negative_A1_4R() {
    chassis.setPose(12 * TeamColorInt, -12, -120);
    LadyBrownOdom.set_position(ArmLoadPos * 100);
    pros::delay(20);

    ArmPIDtoPosition(ScoreAlliancePos, 500);
    chassis.moveToPoint(24 * TeamColorInt, 24, 1600, {.forwards = false, .minSpeed = 50});
    pros::delay(500);
    ArmPIDtoPosition(3.00, 650);

    chassis.waitUntilDone();

    //-----------------------------------------------


    MobileGoal.set_value(4095);
    MogoToggle = -1; // because we are holding the a Mogoal

    pros::delay(250);
    chassis.turnToHeading(45 * TeamColorInt, 800);
    chassis.waitUntilDone();

    chassis.moveToPoint(35 * TeamColorInt, 30.7, 1000);
    
    chassis.swingToHeading(87 * TeamColorInt, DriveSide::RIGHT, 700);
    chassis.waitUntilDone();

    IntakeFlex.move_velocity(200);
    IntakeHook.move_velocity(600);

    chassis.moveToPoint(50 * TeamColorInt, 31, 1000, {.maxSpeed = 60, .minSpeed = 59});
    chassis.waitUntilDone();
    IntakeFlex.brake();
    chassis.moveToPoint(48 * TeamColorInt, 28, 500, {.forwards = false, .minSpeed = 80});

    chassis.swingToHeading(160 * TeamColorInt, DriveSide::RIGHT, 600);
    chassis.turnToHeading(-145 * TeamColorInt, 1000);

    chassis.waitUntilDone();
    IntakeFlex.move_velocity(200);
    chassis.moveToPoint(48 * TeamColorInt, 22, 1200, {.minSpeed = 127});


    chassis.turnToPoint(64 * TeamColorInt, -24, 1000);

    chassis.waitUntilDone();
    IntakeFlex.move_velocity(200);

    chassis.moveToPoint(64 * TeamColorInt, -24, 2000, {.minSpeed = 1000});

    chassis.moveToPoint(59 * TeamColorInt, 0, 1000, {.forwards = false});
    chassis.waitUntilDone();


    chassis.waitUntilDone();
    
}

//--------------------------------------------------

void TB(int PosOrNeg) {
    chassis.turnToPoint(14 * PosOrNeg * TeamColorInt, 30, 1000);
    chassis.moveToPoint(14 * PosOrNeg * TeamColorInt, 30, 2000, {.minSpeed = 40});

    chassis.waitUntilDone();
}

void Negative_PC() {
    chassis.turnToPoint(0 * TeamColorInt, 0, 1000);
    chassis.waitUntilDone();
    IntakeHook.brake();
    IntakeFlex.brake();
    chassis.moveToPoint(-57 * TeamColorInt, -2, 2000);
    pros::delay(800);
    DoinkerLeft.set_value(4096);
    chassis.turnToHeading(-45 * TeamColorInt, 1000);
    DoinkerLeft.set_value(0);

    chassis.waitUntilDone();
}

void A1_TB() {
    chassis.turnToPoint(-2 * TeamColorInt, -8.5, 800);
    chassis.moveToPoint(-2 * TeamColorInt, -8.5, 1500);
    pros::delay(400);
    Arm.move_absolute(ArmLoadPos*10+20, 200);

    doingWallstake = true;
    chassis.waitUntilDone();
    
    // IntakeHook.move_velocity(-400);
    // pros::delay(200);

    chassis.turnToHeading(175 * TeamColorInt, 500);
    chassis.moveToPoint(-0.8 * TeamColorInt, -19, 1000, {.maxSpeed = 80, .minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.setPose(0, -14.5, 180 * TeamColorInt);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -5.8, 1000, {.forwards = false});
    Arm.move_absolute(ScoreAlliancePos * 10 + 100, 200);

    pros::delay(300);
    chassis.moveToPoint(0, 0, 500, {.forwards = false, .minSpeed = 127});
    chassis.turnToHeading(10 * TeamColorInt, 1000, {.maxSpeed = 80});
}

void Negative_MidRingLB() {
    chassis.turnToPoint(10 * TeamColorInt, 37, 800);
    chassis.moveToPoint(10 * TeamColorInt, 37, 700, {.minSpeed = 127});
    chassis.moveToPoint(10 * TeamColorInt, 37, 1500, {.maxSpeed = 60});

    if (TeamColor) {
        DoinkerRight.set_value(4096);
    } else {
        DoinkerLeft.set_value(4096);
    }

    chassis.waitUntilDone();

    chassis.moveToPoint(20 * TeamColorInt, 20, 1000, {.forwards = false}, false);
    DoinkerLeft.set_value(0);
    DoinkerRight.set_value(0);
    IntakeHook.brake();
    pros::delay(300);

    IntakeFlex.move_velocity(200);
    chassis.turnToPoint(17.5 * TeamColorInt, 34, 1000);
    chassis.moveToPoint(17.5 * TeamColorInt, 34, 1000);

    chassis.waitUntilDone();
}

//----------------------------------------------------------------------------------Skills----------------------------------------------------------------------------------

void Skills() {
    chassis.setPose(12 * TeamColorInt, -12, 180);
    chassis.moveToPoint(24 * TeamColorInt, 24, 1000, {.forwards = false});

    chassis.waitUntilDone();
}

//----------------------------------------------------------------------------------Auto Chaining----------------------------------------------------------------------------------

void Negative_RingRush_A1_5R_TB() {
    Negative_RingRush_5R();
    A1_TB();
}

void Negative_RingRush_6R_LBR() {
    Negative_RingRush_5R();
    Negative_MidRingLB();
} 

void Negative_A1_4R_TB() {
    Negative_A1_4R();
    TB(-1);
}
void Negative_A1_4R_PC() {
    Negative_A1_4R();
    Negative_PC();
}

void Negative_RingRush_5R_TB() {
    Negative_RingRush_5R_ALT();
    TB(-1);
}

void SoloAWP_TB() {
    SoloAWP();
    TB(-1); 
}


//----------------------------------------------------------------------------------Auto----------------------------------------------------------------------------------

void autonomous() {
    Positive_GoalRush_3R();
    //Negative_RingRush_A1_5R_TB();
}

//----------------------------------------------------------------------------------opcontrol----------------------------------------------------------------------------------


void opcontrol() { //Driver
    autonomousRunning = false;
    pros::rtos::Task TaskMacros(FuncMacros);
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
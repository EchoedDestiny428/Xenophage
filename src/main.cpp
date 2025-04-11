#include "main.h"
#include "lemlib/api.hpp"

//----------------------------------------------------------------------------------Device Setup----------------------------------------------------------------------------------
//--START Device Setup--

pros::MotorGroup Left ({-20, 13, -11}, pros::MotorGears::blue);
pros::MotorGroup Right ({14, 18, -17}, pros::MotorGears::blue);
pros::Motor Arm (8, pros::MotorGears::green);
pros::Motor IntakeFlex (-2, pros::MotorGears::green);
pros::Motor IntakeHook (-1, pros::MotorGears::blue);

pros::Controller ParaRAID(pros::E_CONTROLLER_MASTER);


pros::adi::DigitalOut Lift('D');
pros::adi::DigitalOut Eject('E');
pros::adi::DigitalOut MobileGoal('B');
pros::adi::DigitalOut Doinker('A');
pros::adi::DigitalOut Hang('F');

pros::Rotation LadyBrownOdom(-3);
pros::Optical VSensor(9); //wrong
pros::Imu Inertial(10); //wrong
pros::Rotation HorizontalEnc(6);
pros::Rotation VerticalEnc(-12);

lemlib::TrackingWheel Horizontal(&HorizontalEnc, lemlib::Omniwheel::NEW_2, 0.0); //change
lemlib::TrackingWheel Vertical(&VerticalEnc, lemlib::Omniwheel::NEW_2, 0.0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&Left, &Right, 12.8, lemlib::Omniwheel::NEW_325, 450, 2); //left right track width, wheel type, rpm, drift

// Forward PID
lemlib::ControllerSettings linearController(8,  // proportional gain (kP)
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
lemlib::ControllerSettings angularController(4.8, // proportional gain (kP)
                                             0.6, // integral gain (kI)
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

double ArmPos() {
    double armPos = LadyBrownOdom.get_position()/100.000;
    return armPos;
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    LadyBrownOdom.reset_position();
    
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

//----------------------------------------------------------------------------------Global Variables----------------------------------------------------------------------------------


bool TeamColor = true; //true = blue, red = false 
bool skillz = false;

//----------------------------------------------------------------------------------Auto Setup----------------------------------------------------------------------------------

bool DontEject = false;
bool EjectStay = false;


void AutoEject() {
  pros::c::optical_rgb_s_t rgb_value;
  while (true) {
    rgb_value = VSensor.get_rgb();
    if (!TeamColor && (rgb_value.blue > 172) && (rgb_value.red < 230)) {
      Eject.set_value(4096);
      pros::delay(500);
    } else if (TeamColor && (rgb_value.red > 230) && (rgb_value.blue < 180)) {
      Eject.set_value(4096);
      pros::delay(500);
    } else if (!EjectStay) {
      Eject.set_value(0);
    }
    pros::delay(5);
  }
}

//----------------------------------------------------------------------------------Auto----------------------------------------------------------------------------------

void autonomous() {


  pros::delay(20);

  //rip no autonomous :)


}

//--------------------------------------------------------------------------------Doinker--------------------------------------------------------------------------------

void DoinkerControl() {
  int DoinkerToggle = 1;
  while (true) {
    if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      if (DoinkerToggle == 1) {
        Doinker.set_value(4096);
      } else {
        Doinker.set_value(0);
      }
      DoinkerToggle *= -1;
      pros::delay(500);
    }
    pros::delay(10);
  }
}

//--------------------------------------------------------------------------------Hang--------------------------------------------------------------------------------

void HangControl() {
  int HangToggle = 1;
  while (true) {
    if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      if (HangToggle == 1) {
        Hang.set_value(4096);
      } else {
        Hang.set_value(0);
      }
      HangToggle *= -1;
      pros::delay(300);
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
      chassis.moveToPoint(0, -6, 800, {.minSpeed = 127});
      chassis.waitUntilDone();
    }

    pros::delay(20); // Run for 20 ms then update
  }
}

//--------------------------------------------------------------------------------Arm--------------------------------------------------------------------------------



void ArmPIDtoPosition(double target) {
    double ArmKp = 4.00; // Proportional Modifier
    double ArmKd = 3.20; // Derivative Modifier
    double error;
    double prevError = 0;
    double derivative = 0;

    while ((ArmPos() > (target + 0.4)) || (ArmPos() < (target - 0.4))) {
        error = target - ArmPos();
        derivative = error - prevError;

        pros::lcd::print(6, "-----------Arm Error: %f", error);

        Arm.move_velocity((ArmKp * error) + (ArmKd * derivative)); // PID Control


        prevError = error;
        
        pros::delay(20);

        if ((ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && (ArmPos() > 5)) {
            break;
        }
    }
    Arm.brake();
    pros::delay(100);
    if ((ArmPos() > (target + 0.4)) || (ArmPos() < (target - 0.4))) {
        ArmKp = 8.00;
        ArmPIDtoPosition(target);
    } else {
        Arm.brake();
        ArmKp = 4.00;
    }
}

void ArmControl() {
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    //LadyBrownOdom.reverse();

    double ArmLoadPos = 26.00;
    double ArmTipPos = 180.00;
  
    while (true) { 
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            ArmPIDtoPosition(ArmTipPos);
            
        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (ArmPos() < (ArmLoadPos-10.0))) {
            ArmPIDtoPosition(ArmLoadPos);

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            ArmPIDtoPosition(5.00);
            
        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            Arm.move_velocity(200);

        } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (ArmPos() > 5) {
                Arm.move_velocity(-200);
            } else {
                Arm.brake();
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
                IntakeFlex.move_relative(-30, 180);
                IntakeHook.move_relative(-30, 180);
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
    pros::c::optical_rgb_s_t rgb_value;
    while (true) {
        rgb_value = VSensor.get_rgb();
        if (!TeamColor && (rgb_value.blue > 180) && (rgb_value.red < 230) && (MogoToggle == -1)) {
            Eject.set_value(4096);
            pros::delay(500);
        } else if (TeamColor && (rgb_value.red > 230) && (rgb_value.blue < 180) && (MogoToggle == -1)) {
            Eject.set_value(4096);
            pros::delay(500);
        } else if (!EjectStay) {
            Eject.set_value(0);
        }
        pros::delay(5);
    }
}



//----------------------------------------------------------------------------------opcontrol----------------------------------------------------------------------------------


void opcontrol() { //Driver
    pros::rtos::Task TaskChassisControl(ChassisControl);
    pros::rtos::Task TaskArmControl(ArmControl);
    pros::rtos::Task TaskFuncIntake(FuncIntake);
    pros::rtos::Task TaskFuncMogo(FuncMogo);
    pros::rtos::Task TaskEject(DriverEject);
    pros::rtos::Task TaskDoiner(DoinkerControl);
    pros::rtos::Task TaskHang(HangControl);
    pros::rtos::Task TaskFuncIntakeLift(FuncIntakeLift);



    
    while (true) {
        

		pros::delay(20);                               // Run for 20 ms then update
    }
}
#include "main.h"
#include "lemlib/api.hpp"




pros::MotorGroup Left ({8, -9, -10}, pros::MotorGears::blue);
pros::MotorGroup Right ({3, -4, 5}, pros::MotorGears::blue);
pros::Motor ArmLeft (11, pros::MotorGears::green);
pros::Motor ArmRight (12, pros::MotorGears::green);
pros::Motor IntakeFlex (-16, pros::MotorGears::green);
pros::Motor IntakeHook (-21, pros::MotorGears::green);

pros::Controller ParaRAID(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalIn ArmChecker('C');
pros::adi::DigitalOut Lift('D');
pros::adi::DigitalOut Eject('E');
pros::adi::DigitalOut MobileGoal('A');
pros::adi::DigitalOut Doinker('B');
pros::adi::DigitalOut Hang('F');

pros::Optical VSensor(4);

pros::Imu Inertial(13);
pros::Rotation HorizontalEnc(6);
pros::Rotation VerticalEnc(1);

lemlib::TrackingWheel Horizontal(&HorizontalEnc, lemlib::Omniwheel::NEW_2, -4.5); //change
lemlib::TrackingWheel Vertical(&VerticalEnc, lemlib::Omniwheel::NEW_2, 1.25);

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

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}



void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

void disabled() {  //well disable on bot enable

 
}


void competition_initialize() { //Auto Selector


}

bool intakeStop = true;

void StopIntake() {
  if (VSensor.get_proximity() > 200 && intakeStop == true) {
    IntakeFlex.brake();
    IntakeHook.brake();
    intakeStop = false;
  }
}

bool TeamColor = true; //true = blue, red = false 
bool skillz = false;

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



void autonomous() { //Auto  -------------------------------------CHECK AUTO COMMENT, TEAM COLOR, TOP DIST THINGY
  //pros::rtos::Task TaskStopIntake(StopIntake);
  //pros::rtos::Task TaskAutoEject(AutoEject); //TURN OFF FOR SKILLS!!!!!!!!!!11-----------------------------------------------------------

  //pros::rtos::Task TaskJamPrev(intakeAutoJamPrev);
  //VSensor.set_led_pwm(100);

  pros::delay(20);

  //rip no autonomous :)


}


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
  }
}

void ArmControl() {
  ArmLeft.tare_position();
  ArmRight.tare_position();
  ArmLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ArmRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  int ArmLoadPos = 258;
  int ArmTipPos = 1600;

  while (true) { 
    if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      ArmLeft.move_absolute(ArmTipPos, 200);
      ArmRight.move_absolute(ArmTipPos, 200);
      while (((ArmLeft.get_position() + ArmRight.get_position())/2 < (ArmTipPos - 2)) || ((ArmLeft.get_position() + ArmRight.get_position())/2 > (ArmTipPos + 2))) {
        pros::delay(5);
      }
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && ArmChecker.get_value()) {
      ArmLeft.move_absolute(ArmLoadPos, 100);
      ArmRight.move_absolute(ArmLoadPos, 100);
      while (((ArmLeft.get_position() + ArmRight.get_position())/2 < (ArmLoadPos - 2)) || ((ArmLeft.get_position() + ArmRight.get_position())/2 > (ArmLoadPos + 2))) {
        pros::delay(10);
      }
    } else if (ArmChecker.get_value()) {
      ArmLeft.brake();
      ArmRight.brake();
      ArmLeft.tare_position();
      ArmRight.tare_position();
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      ArmLeft.move_velocity(-200);
      ArmRight.move_velocity(-200);
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      while (!ArmChecker.get_value()) {
        ArmLeft.move_velocity(-200);
        ArmRight.move_velocity(-200);
        pros::delay(5);
        if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
          goto ArmStuck;
        }
      }

      ArmLeft.move_relative(-100, 200);
      ArmRight.move_relative(-100, 200);
      pros::delay(300);
      ArmLeft.brake();
      ArmRight.brake();
      ArmLeft.tare_position();
      ArmRight.tare_position();

      ArmStuck:
      ArmLeft.brake();
      ArmRight.brake();
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      ArmLeft.move_absolute(ArmLoadPos, 200);
      ArmRight.move_absolute(ArmLoadPos, 200);
      while (((ArmLeft.get_position() + ArmRight.get_position())/2 < (ArmLoadPos - 5)) || ((ArmLeft.get_position() + ArmRight.get_position())/2 > (ArmLoadPos + 5))) {
        pros::delay(5);
      }
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      ArmLeft.move_velocity(200);
      ArmRight.move_velocity(200);
    } else {
      ArmLeft.brake();
      ArmRight.brake();
      pros::delay(5);
    }
    pros::delay(10);
  }
}



void FuncIntake() {
  int IntakeToggle = 1;
  int IntakeSpeed = 100; //100 for skills, 90 regular
  while (true) {
    if (IntakeHook.get_actual_velocity() == 0 && IntakeToggle == -1) {
      IntakeHook.move_velocity(IntakeSpeed * -2);
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      while (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        IntakeFlex.move_velocity(IntakeSpeed * -2);
        IntakeHook.move_velocity(IntakeSpeed * -2);
      }
      if (IntakeToggle == 1) {
        IntakeFlex.brake();
        IntakeHook.brake();
      } else {
        IntakeFlex.move_velocity(IntakeSpeed * 2);
        IntakeHook.move_velocity(IntakeSpeed * 1.8);
      }
    } else if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      if (IntakeToggle == 1) {
        IntakeFlex.move_velocity(IntakeSpeed * 2);
        IntakeHook.move_velocity(IntakeSpeed * 1.8);
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
      IntakeHook.move_velocity(IntakeSpeed * 1.8);
      pros::delay(10);
    }
    pros::delay(10);
  }
}

void FuncIntakeLift() {
  int IntakeLiftToggle = 1;
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
}

int MogoToggle = 1;
void FuncMogo() {
  while (true) {
    if (ParaRAID.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      if (MogoToggle == 1) {
        MobileGoal.set_value(4095);
        ParaRAID.set_text(0, 0, "On ");
      } else {
        MobileGoal.set_value(0);
        //Eject.set(false);
        ParaRAID.set_text(0, 0, "Off");
      }

      MogoToggle *= -1;
      pros::delay(500);
    }
    pros::delay(5);
  }
}

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






void opcontrol() { //Driver
	pros::rtos::Task TaskChassisControl(ChassisControl);
  pros::rtos::Task TaskArmControl(ArmControl);
  pros::rtos::Task TaskFuncIntake(FuncIntake);
  pros::rtos::Task TaskFuncMogo(FuncMogo);
  pros::rtos::Task TaskEject(DriverEject);
  pros::rtos::Task TaskDoiner(DoinkerControl);
  pros::rtos::Task TaskHang(HangControl);
  pros::rtos::Task TaskFuncIntakeLift(FuncIntakeLift);



  pros::c::optical_rgb_s_t rgb_value;
  VSensor.set_led_pwm(100);
	while (true) {
    rgb_value = VSensor.get_rgb();
    pros::screen::print(TEXT_MEDIUM, 1, "Red value: %lf \n", rgb_value.red);
    pros::screen::print(TEXT_MEDIUM, 2, "Green value: %lf \n", rgb_value.green);
    pros::screen::print(TEXT_MEDIUM, 3, "Blue value: %lf \n", rgb_value.blue);
    pros::screen::print(TEXT_MEDIUM, 4, "Heading: %lf \n", chassis.getPose().theta);

    //pros::screen::print(TEXT_MEDIUM, 4, "Proximity value: %ld \n", VSensor.get_proximity());
    pros::delay(20);
  }
}
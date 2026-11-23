#include "main.h"
#include "autons.h"
#include <stdlib.h>
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Distance liftDistance(16);

pros::MotorGroup right_motor_group({12,13}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::MotorGroup left_motor_group({-19,-20}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

//pros::MotorGroup right_motor_group({21,-10,1}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
//pros::MotorGroup left_motor_group({-9,8,2}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);


pros::Distance clampDistance(4);
pros::MotorGroup lift({17,-18,15}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::ADIDigitalOut clampPiston('A');

pros::Optical rollerOpticalRight(1);
pros::Optical rollerOpticalLeft(6);

pros::Motor roller(11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);


// roller team color
bool blue = false;

// check if hues match certain color
bool checkBlue(double hue){
	return (hue<220 && hue>208);
}
bool checkRed(double hue){
	return ( hue<15 || hue>342);
}
bool checkColor(double hue){
	if (blue){
		return checkBlue(hue);
	}
	else{
		return checkRed(hue);
	}
}
double lowEndGain = 0;
double driverCurve(double stick) {
  if (fabs(stick) < 3) {
    return 0;
  }
  if (5 <= 0) {
    return stick / 127.0;
  }
  lowEndGain = exp(-5 / 10.0);
  return (12000 / 127.0) * ((lowEndGain + exp((fabs(stick) - 127) / 10.0) * (1 - lowEndGain)) * stick);
}


lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(8);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(-14);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-9);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 2, 4.122);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2, -3.333);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(17, // proportional gain (kP)16
                                              0, // integral gain (kI)
                                              105, // derivative gain (kD)92
                                              0, // anti windup
                                              1, // small error range, in inches 1
                                              20, // small error range timeout, in milliseconds 20
                                              3, // large error range, in inches 3
                                              500, // large error range timeout, in milliseconds 500
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(10, // proportional gain (kP)8
                                              0, // integral gain (kI)
                                              61, // derivative gain (kD)47
                                              0, // anti windup
                                              2, // small error range, in degrees0
                                              20, // small error range timeout, in milliseconds20
                                              3, // large error range, in degrees 3
                                              500, // large error range timeout, in milliseconds 500
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

tr_sensor north({1+(1/16), -3.75}, 10);
/**
 * East sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor east({5+(13/16), 1+(1/16)}, 2);
/**
 * South sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor south({4.861, 5.25}, 3);
/**
 * West sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor west({5.7, -0.75-(1/32)}, 21);
tr_chassis dsr_system(&chassis, {&north, &east, &south, &west}, tr_fields::plastic);

double liftTaskTarget =0;
//autonamous lift control 
void liftTask(){
	while(true){
		
		pros::lcd::set_text(0, std::to_string(chassis.getPose().x));
		pros::lcd::set_text(1, std::to_string(chassis.getPose().y));
		pros::lcd::set_text(2, std::to_string(chassis.getPose().theta));
		
		if(liftTaskTarget!=0){
			if(liftTaskTarget>liftDistance.get_distance()){
				while(liftTaskTarget>liftDistance.get_distance()){
					lift.move_voltage(12000);
					pros::delay(10);
				}
				lift.move_voltage(0);
				liftTaskTarget=0;
			}
			else{
				while(liftTaskTarget<liftDistance.get_distance()){
					lift.move_voltage(-6000);
					pros::delay(10);
				}
				lift.move_voltage(0);
				liftTaskTarget=0;
			}
		}
		
		
		pros::delay(10);
	}
}



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 // correct for imu error
double imuScale = 1.01198; 
void imuScaleTask() {
    double prev = imu.get_rotation();  

    while (true) {
        double now = imu.get_rotation();

        if (now != PROS_ERR_F) {
            double delta = now - prev;
            prev = now;

            lemlib::Pose p = chassis.getPose();
            chassis.setPose(p.x, p.y, p.theta + delta * (imuScale - 1.0));
        }
        pros::delay(10);
    }
}

void initialize() {
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");
	lift.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
	rollerOpticalLeft.set_led_pwm(100);
	rollerOpticalLeft.set_integration_time(3);
	rollerOpticalRight.set_led_pwm(100);
	rollerOpticalRight.set_integration_time(3);
	

	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	
	chassis.calibrate();
	//dsr_system.perform_dsr_init(tr_quadrant::POS_NEG, 180);
	chassis.setPose(0,0,180);
	pros::Task scaleTask(imuScaleTask);
	pros::Task liftTaskTask(liftTask);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	
	//skills();
	matchAuto();
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

int liftI = 0;
bool homing = false;
bool clamped = false;

void opcontrol() {
	
	/*left_motor_group.move_voltage(2000);
	right_motor_group.move_voltage(2000);
	lift.tare_position_all();


	//pros::delay(1000);
	//master.rumble("-");
	//liftTaskTarget=55;
	liftTaskTarget=150;
	roller.tare_position();
	roller.move_relative(3,600);
	while(!checkColor(rollerOpticalRight.get_hue())&&roller.get_target_velocity()!=0){
		//pros::lcd::set_text(3, std::to_string(roller.get_current_draw()));
		if(roller.get_position()>2.9){
			roller.tare_position();
			roller.move_relative(3,600);
		}
		pros::delay(10);
	}
	//pros::delay(100);//if red
	
	roller.move_voltage(0);
	chassis.moveToPoint(5,14.8,1000,{.forwards=false,.minSpeed=10,.earlyExitRange=7},false);
	chassis.turnToPoint(16.43,13.606,1000,{.forwards=false,.minSpeed=70,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,13.606,1100,{.forwards=false,.maxSpeed=70,.minSpeed=70},true);
	pros::delay(600);
	
	liftTaskTarget=90;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(100);
	goalReset(16.43,13.606);
	
	chassis.moveToPoint(10,26,2000,{},false);
	lift.move_absolute(0,400);
	//chassis.turnToHeading(225,1000,{.minSpeed=10,.earlyExitRange=2},false);
	chassis.turnToPoint(16,37.4,1000,{.forwards=false},false);
	chassis.moveToPoint(16,37.4,1000,{.forwards=false,.maxSpeed=75,.minSpeed=15,.earlyExitRange=10},false);
	
	chassis.moveToPoint(16,37.4,1000,{.forwards=false,.maxSpeed=30},true);
	x=0;
	while(clampDistance.get_distance()>50.8&&x<200){
		pros::delay(10);
		x++;
	}
	clampPiston.set_value(false);
	pros::delay(200);
	liftTaskTarget=175;
	chassis.cancelMotion();
	chassis.turnToHeading(0,1000,{.minSpeed=2,.earlyExitRange=1},false);
	chassis.moveToPoint(16.43,13.606,1000,{.forwards=false,.maxSpeed=92,.minSpeed=40,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,13.606,600,{.forwards=false,.maxSpeed=50},true);
	pros::delay(300);
	liftTaskTarget=85;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(100);
	goalReset(16.43,13.606);
	
	//chassis.moveToPoint(34,36,1500,{},false);
	chassis.turnToPoint(27,28,1000,{.maxSpeed=100,.minSpeed=50,.earlyExitRange=1},false);
	chassis.moveToPoint(27,28,1000,{.minSpeed=10,.earlyExitRange=1},false);
	
	chassis.turnToPoint(41,12.943,1000,{.forwards=false},false);
	lift.move_absolute(0,400);
	chassis.moveToPoint(41,12.943,1000,{.forwards=false,.maxSpeed=65,.minSpeed=15,.earlyExitRange=10},false);
	chassis.moveToPoint(41,12.943,2000,{.forwards=false,.maxSpeed=30},true);
	x=0;
	while(clampDistance.get_distance()>50.8&& x<200){
		pros::delay(10);
		master.rumble(".");
		x++;
	}
	clampPiston.set_value(false);
	pros::delay(200);
	
	liftTaskTarget=360;
	chassis.cancelMotion();
	chassis.turnToPoint(16.43,13.606,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=30},false);
	chassis.turnToHeading(90,1000,{.minSpeed=2,.earlyExitRange=2});
	chassis.moveToPoint(16.43,13.606,1000,{.forwards=false,.maxSpeed=92,.minSpeed=40,.earlyExitRange=16},false);
	chassis.moveToPoint(16.43,13.606,600,{.forwards=false,.maxSpeed=50},true);
	pros::delay(300);
	lift.move_relative(-1.3,400);
	pros::delay(300);
	clampPiston.set_value(true);
	pros::delay(100);
	goalReset(16.43,13.606);
	lift.move_absolute(0,400);
	homing = true;
	clamped=true;*/

	while (true) {

        //drive
        left_motor_group.move_voltage(driverCurve(master.get_analog(ANALOG_LEFT_Y)));
        right_motor_group.move_voltage(driverCurve(master.get_analog(ANALOG_RIGHT_Y)));
        
        //roller
		/*if(!blue){
        if(master.get_digital(DIGITAL_Y)&&!(checkRed(rollerOpticalLeft.get_hue())||checkRed(rollerOpticalRight.get_hue()))){
            roller.move_voltage(12000);
        }
        else{
            roller.move_voltage(0);
        }
		}
		else{
			if(master.get_digital(DIGITAL_Y)&&!(checkBlue(rollerOpticalLeft.get_hue())||checkBlue(rollerOpticalRight.get_hue()))){
            	roller.move_voltage(12000);
        	}
        	else{
            	roller.move_voltage(0);
        	}
		}*/
		if(master.get_digital(DIGITAL_Y)){
            roller.move_voltage(12000);
        }
        else{
            roller.move_voltage(0);
        }


		//seperate clamp
		if(master.get_digital_new_press(DIGITAL_RIGHT)){
			clamped=!clamped;
			clampPiston.set_value(clamped);
		}

        //lift
        if(master.get_digital(DIGITAL_A)){
			lift.move_relative(1.5,600);
			pros::delay(500);
			clampPiston.set_value(false);
			lift.move_relative(2.5,600);
			homing=true;
		}
		if(master.get_digital(DIGITAL_B)){
			lift.move_relative(1.5,600);
			pros::delay(500);
			clampPiston.set_value(false);
			lift.move_relative(2.5,600);
			homing=true;
		}

        if(master.get_digital(DIGITAL_L1)&&clampDistance.get_distance()<50.8&& clamped){
            clampPiston.set_value(false);
            clamped=false;
            liftI=1;
        }
        if(liftI>0 && liftI<6){
            liftI++;
        }
        if(liftI==6){
            if(clampDistance.get_distance()>40){
                liftI = 0;
                clamped=true;
            }
            else{
            lift.move_absolute(0.75,600);
            }
            
        }
        if(liftI == 6 && lift.get_position()>0.7){
            liftI=0;
        }

		// score and return to home.
        if(master.get_digital_new_press(DIGITAL_L2)){
            clampPiston.set_value(true);
            clamped=true;
			lift.move_absolute(0,-400);
            homing = true;
        }

        
        
        // manual controls
        if(liftI!=6){
        if(master.get_digital(DIGITAL_R1)){
            lift.move_voltage(12000);
        }
        else if(master.get_digital(DIGITAL_R2)){
            lift.move_voltage(-8000);
        }
        else if (!homing){
            lift.move_voltage(0);
        }
        else if(liftDistance.get_distance()<15){
            lift.tare_position_all();
            homing=false;
            if(lift.get_voltage()<0){
                lift.move_voltage(0);
            }
        }
        }
		pros::delay(10);         
	}
}
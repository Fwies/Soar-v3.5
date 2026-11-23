#include "main.h"
double l = 6.75;
void goalReset(double x,double y){
	chassis.setPose(x + (sin(chassis.getPose(true).theta)*l), y+(cos(chassis.getPose(true).theta)*l), chassis.getPose().theta);
}

double goalX(){
	return chassis.getPose().x - l * std::sin(chassis.getPose(true).theta);
}
double goalY(){
	return chassis.getPose().y - l * std::cos(chassis.getPose(true).theta);
}



int x = 0;
void matchAuto(){
	left_motor_group.move_voltage(2000);
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
	clamped=true;
	pros::delay(100);
	goalReset(16.43,13.606);
	chassis.turnToPoint(26,41,1000,{.minSpeed=2,.earlyExitRange=4},false);
	lift.move_absolute(0,400);

	pros::delay(999999);
	chassis.moveToPoint(26,41,1000,{.minSpeed=10,.earlyExitRange=12},false);
	chassis.swingToPoint(57,49,DriveSide::RIGHT,1000,{.minSpeed=2,.earlyExitRange=5},false);
	chassis.moveToPoint(57,49,1500,{},false);
	chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
	left_motor_group.move_voltage(2000);
	right_motor_group.move_voltage(2000);
	roller.tare_position();
	roller.move_relative(3,600);
	while(!checkColor(rollerOpticalLeft.get_hue())&&roller.get_target_velocity()!=0){
		//pros::lcd::set_text(3, std::to_string(roller.get_current_draw()));
		if(roller.get_position()>2.9){
			roller.tare_position();
			roller.move_relative(3,600);
		}
		pros::delay(10);
	}
	chassis.moveToPoint(50,50,1500,{.forwards=false,.minSpeed=127},false);
	

	//pros::delay(99999);
	/*chassis.moveToPoint(28,13.6,1000,{},false);
	lift.move_absolute(0,500);
	
	chassis.turnToPoint(16.43,-9,1000,{.forwards=false},false);
	clampPiston.set_value(false);
	chassis.moveToPoint(16.43,-9,1000,{.forwards=false,.maxSpeed=127,.minSpeed=30,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,-9,1000,{.forwards=false,.maxSpeed=60,.minSpeed=60,.earlyExitRange=13},false);
	clampPiston.set_value(true);
	//chassis.moveToPoint(-18,8,2000,{.forwards=false,.maxSpeed=120,.minSpeed=70},true);
	left_motor_group.move_voltage(-12000);
	while(clampDistance.get_distance()>50.8){
		pros::delay(10);
		master.rumble(".");
	}
	clampPiston.set_value(false);
	//chassis.moveToPoint(16.43,1,200,{},false);
	//chassis.moveToPoint(16.43,-9,200,{.forwards=false},false);
	left_motor_group.move_voltage(0);
	pros::delay(200);
	liftTaskTarget=560;
	left_motor_group.move_voltage(12000);
	right_motor_group.move_voltage(-6000);
	pros::delay(700);
	
	chassis.turnToPoint(16.43,13.606,1000,{.forwards=false,.minSpeed=2,.earlyExitRange=2},false);
	chassis.moveToPoint(16.43,13.606,500,{.forwards=false},false);*/
	/*clampPiston.set_value(true);
	liftTaskTarget=710;*/
	//liftTaskTarget=450;
}

void skills(){
	left_motor_group.move_voltage(2000);
	right_motor_group.move_voltage(2000);
	lift.tare_position_all();


	//pros::delay(1000);
	//master.rumble("-");
	//liftTaskTarget=55;
	liftTaskTarget=150;
	roller.tare_position();
	roller.move_relative(2.5,600);
	while(!checkRed(rollerOpticalRight.get_hue())&&roller.get_target_velocity()!=0){
		//pros::lcd::set_text(3, std::to_string(roller.get_current_draw()));
		if(roller.get_position()>2.4){
			roller.tare_position();
			roller.move_relative(2.5,600);
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
	//chassis.turnToPoint(11,26,2000,{.minSpeed=2,.earlyExitRange},false);
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
	chassis.moveToPoint(27,28,1000,{.minSpeed=10,.earlyExitRange=2},false);
	
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
	//pre titan reset

	dsr_system.perform_dsr_quad(tr_quadrant::POS_NEG);
	double goalPosX = goalX();
	double goalPosY = goalY();
	//pros::delay(99999); //58, -62

	double liftCup[5] = {550,740,930,0,0};
	int i =0;
	while(i!=3){
	chassis.moveToPoint(58,goalPosY,2000,{},true);
	chassis.waitUntil(3);

	lift.move_absolute(0,400);
	chassis.waitUntilDone();
	chassis.turnToPoint(58,-66, 1000,{.forwards=false},false);
	chassis.moveToPoint(58,-66, 2000,{.forwards=false,.maxSpeed=85,.minSpeed=10},true);
	chassis.waitUntil(5);
	chassis.cancelMotion();
	chassis.moveToPoint(58,-66, 2000,{.forwards=false,.maxSpeed=50},true);
	x=0;
	/*while(clampDistance.get_distance()>50.8&&x<300){
		pros::delay(10);
		//master.rumble(".");
		x++;
	}*/
		
	while(true){
		if(left_motor_group.get_actual_velocity()>-50){
			break;
		}
		pros::delay(10);
	}
	
	clampPiston.set_value(false);
	chassis.cancelMotion();
	pros::delay(200);
	
	/*chassis.moveToPoint(55,12, 100,{},false);
	chassis.moveToPoint(56, 3.5, 200, {.forwards=false},false);*/
	//pros::delay(200);
	///chassis.cancelMotion();
	//chassis.swingToPoint(40,12,DriveSide::LEFT,1000,{.minSpeed=30,.earlyExitRange=20},false);
	//	liftTaskTarget=liftCup[i];
	//chassis.moveToPoint(40,12,2000,{/*.minSpeed=10,.earlyExitRange=15*/},false);

	//chassis.turnToPoint(18,12,2000,{.forwards=false/*,.minSpeed=50,.earlyExitRange=30*/},false);
	//chassis.moveToPoint(18,12,1000,{.forwards=false,.maxSpeed=50},false);
	chassis.moveToPoint(58,goalPosY,1000,{.minSpeed=10,.earlyExitRange=12},true);
	chassis.waitUntil(3);
	liftTaskTarget=liftCup[i];
	chassis.waitUntilDone();
	//chassis.turnToPoint(goalPosX,goalPosY,1000,{.forwards=false,.minSpeed=2,.earlyExitRange=1},false);
	chassis.turnToHeading(90,1000,{.minSpeed=2,.earlyExitRange=1},false);
	while(liftTaskTarget-liftDistance.get_distance()>550){
		pros::delay(10);
	}
	chassis.moveToPoint(goalPosX,goalPosY,1000,{.forwards=false,.maxSpeed=80,.minSpeed=15,.earlyExitRange=16},false);
	chassis.moveToPoint(goalPosX,goalPosY,900,{.forwards=false,.maxSpeed=50},true);
	pros::delay(500);
	lift.move_relative(-1.3,400);//liftTaskTarget=liftCup[i]100;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);
	dsr_system.perform_dsr_quad(tr_quadrant::POS_NEG);
	goalPosX = goalX();
	goalPosY = goalY();
	//goalReset(24,-48);
	i++;
 	}

	chassis.moveToPoint(57.5,goalPosY,2000,{},true);
	chassis.waitUntil(3);
	lift.move_absolute(-0.1,400);
	chassis.waitUntilDone();
	chassis.turnToPoint(57.5,-64, 1000,{.forwards=false},false);
	chassis.moveToPoint(57.5,-64, 1000,{.forwards=false,.maxSpeed=70,.minSpeed=10,.earlyExitRange=12},false);
	chassis.moveToPoint(57.5,-66, 700,{.forwards=false,.maxSpeed=60},false);
	clampPiston.set_value(false);//42.2,35.5
	//pros::delay(200);
	//roller.move_relative(1.25,600);

	chassis.moveToPoint(57.5,-66,200,{.forwards=false},false);
	chassis.moveToPoint(57.5,-58,100,{},false);
	chassis.moveToPoint(57.5,-66,100,{.forwards=false},false);
	//pros::delay(200);
	chassis.swingToPoint(48,-42,DriveSide::LEFT,1000,{.minSpeed=15,.earlyExitRange=15},false);
	chassis.moveToPoint(48,-42,2000,{},false);
	liftTaskTarget=150;
	chassis.turnToPoint(48,-24,2000,{.forwards=false,},false);
	chassis.moveToPoint(48,-24,1000,{.forwards=false},true);
	chassis.waitUntil(3);
	chassis.cancelMotion();
	chassis.moveToPoint(48,-24,1600,{.forwards=false,.maxSpeed=50},true);
	pros::delay(400);
	liftTaskTarget=80;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);
	liftTaskTarget=90;
	pros::delay(300);
	clampPiston.set_value(false);
	pros::delay(200);
	liftTaskTarget=200;
	pros::delay(300);
	dsr_system.perform_dsr_quad(POS_NEG);
	goalPosX=goalX();
	goalPosY=goalY();
	


	liftCup[0] = 175;
	liftCup[1]= 360;
	liftCup[2] = 550;
	liftCup[3] = 740;
	liftCup[4] = 830;
	i=0;
	while(i<3){
	
	
	
	if(i==0){
		chassis.moveToPoint(goalPosX,-38,1000,{},false);
		chassis.turnToHeading(90,1000,{.minSpeed=10,.earlyExitRange=2},false);
		clampPiston.set_value(true);
		pros::delay(300);
		
		chassis.turnToPoint(55.5,-48,1000,{.minSpeed=2,.earlyExitRange=1},false);
		lift.move_absolute(0,400);
		chassis.moveToPoint(55.5,-48,1000,{},false);

	}
	else{
	chassis.swingToPoint(55.5,-48,DriveSide::LEFT,1000,{.minSpeed=15,.earlyExitRange=15},false);
	lift.move_absolute(0,400);
	
		//lift.move_absolute(0,400);
		chassis.moveToPoint(55.5,-48,2000,{},false);
		
	}
		chassis.turnToPoint(55.5,-67, 2000,{.forwards=false},false);
	
	
	chassis.moveToPoint(55.5,-67, 2000,{.forwards=false,.maxSpeed=85,.minSpeed=10},true);
	chassis.waitUntil(5);
	chassis.cancelMotion();
	chassis.moveToPoint(55.5,-67, 2000,{.forwards=false,.maxSpeed=50},true);
	x=0;
	/*while(clampDistance.get_distance()>50.8&&x<300){
		pros::delay(10);
		//master.rumble(".");
		x++;
	}*/
		
	while(true){
		if(left_motor_group.get_actual_velocity()>-50){
			break;
		}
		pros::delay(10);
	}
	
	clampPiston.set_value(false);
	chassis.cancelMotion();
	pros::delay(200);
	
	pros::delay(200);
	chassis.cancelMotion();
	chassis.swingToPoint(goalPosX-2,-42,DriveSide::LEFT,1000,{.minSpeed=15,.earlyExitRange=15},false);
	liftTaskTarget=liftCup[i];
	chassis.moveToPoint(goalPosX-2,-42,2000,{},false);
	
	chassis.turnToPoint(goalPosX,goalPosY,2000,{.forwards=false,.minSpeed=2,.earlyExitRange=2},false);
	chassis.moveToPoint(goalPosX,goalPosY,1000,{.forwards=false},true);
	chassis.waitUntil(3);
	chassis.cancelMotion();
	chassis.moveToPoint(goalPosX,goalPosY,800,{.forwards=false,.maxSpeed=50},true);
	pros::delay(400);
	lift.move_relative(-1.3,400);//liftTaskTarget=liftCup[i]-100;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);
	dsr_system.perform_dsr_quad(POS_NEG);
	goalPosX=goalX();
	goalPosY=goalY();
	pros::delay(100);

	//goalReset(48,-24);
	
	
	i++;

	}
	chassis.turnToPoint(32,-36,2000,{.minSpeed=70,.earlyExitRange=15},false);
	lift.move_absolute(0,400);
	chassis.moveToPoint(32,-36,2000,{.minSpeed=10,.earlyExitRange=5},false);
	chassis.turnToPoint(32,-6,2000,{.forwards=false,.minSpeed=50,.earlyExitRange=20},false);
	chassis.moveToPoint(32,-6,3000,{.forwards=false},false);
	chassis.turnToPoint(70,-6,2000,{.minSpeed=70,.earlyExitRange=15},false);
	chassis.moveToPoint(70,-6,1200,{},false);
	chassis.setBrakeMode(MOTOR_BRAKE_HOLD);

	roller.tare_position();
	roller.move_relative(2.5,600);
	while(!checkRed(rollerOpticalRight.get_hue())&&roller.get_target_velocity()!=0){
		//pros::lcd::set_text(3, std::to_string(roller.get_current_draw()));
		if(roller.get_position()>2.4){
			roller.tare_position();
			roller.move_relative(2.5,600);
		}
		pros::delay(10);
	}
	//pros::delay(200);//if red
	
	roller.move_voltage(0);
	chassis.moveToPoint(16,0,2500,{.forwards=false},false);
	

	
	

	
	

	/*chassis.turnToHeading(45, 1000,{},false);
	chassis.turnToHeading(135, 1000,{},false);
	chassis.turnToHeading(315, 1000,{},false);*/
	//chassis.moveToPoint(0,48,4000,{},false);
	//chassis.moveToPoint(0,0,4000,{.forwards=false},false);
}
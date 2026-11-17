#include "main.h"
double l = 6.75;
void goalReset(double x,double y){
	chassis.setPose(x + (sin(chassis.getPose(true).theta)*l), y+(cos(chassis.getPose(true).theta)*l), chassis.getPose().theta);
}





int x = 0;
void matchAuto(){
	//reset position and lift for first pin
	lift.tare_position_all();
    liftTaskTarget=150;

	//spin roller to desired color.
	roller.move_relative(2.5,600);
	
	if(!blue){
	while(!checkRed(rollerOpticalRight.get_hue())&&roller.get_target_velocity()!=0){
		pros::lcd::set_text(3, std::to_string(roller.get_current_draw()));
		if(roller.get_current_draw()<300){
			roller.move_relative(2.5,600);
		}
		pros::delay(10);
	}
	}
	else{
		while(!checkBlue(rollerOpticalRight.get_hue())){
		pros::delay(10);
	}
	}
	roller.move_voltage(0);

	// move to first goal
	chassis.moveToPoint(-5,-14.8,1000,{.forwards=false,.minSpeed=10,.earlyExitRange=7},false);
	chassis.turnToPoint(-18,-14.8,1000,{.forwards=false,.minSpeed=70,.earlyExitRange=15},false);
	chassis.moveToPoint(-18,-14.8,700,{.forwards=false,.maxSpeed=50},false);
	
	// lower and un clamp
	liftTaskTarget=90;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);

	// move to next pin and lower lift
	chassis.moveToPoint(-10,-28,2000,{},false);
	lift.move_absolute(0,500);
	chassis.turnToPoint(-18.07,-39.57,1000,{.forwards=false},false);
	chassis.moveToPoint(-18.07,-39.57,1000,{.forwards=false,.maxSpeed=65,.minSpeed=15,.earlyExitRange=10},false);
	chassis.moveToPoint(-18.07,-39.57,1000,{.forwards=false,.maxSpeed=30},true);

	//auto clamp
	x=0;
	while(clampDistance.get_distance()>50.8&&x<100){
		pros::delay(10);
		x++;
	}
	clampPiston.set_value(false);
	pros::delay(200);
	liftTaskTarget=175;
	chassis.cancelMotion();
	chassis.turnToPoint(-18,-15,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=30},false);
	chassis.moveToPoint(-18,-15,1000,{.forwards=false,.maxSpeed=82,.minSpeed=40,.earlyExitRange=12},false);
	chassis.moveToPoint(-18,-15,600,{.forwards=false,.maxSpeed=50},false);
	liftTaskTarget=85;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(100);
	//chassis.moveToPoint(-34,-36,1500,{},false);
	chassis.turnToPoint(-24,-26,1000,{.maxSpeed=100,.minSpeed=50,.earlyExitRange=10},false);
	chassis.moveToPoint(-24,-26,1000,{.maxSpeed=100,.minSpeed=15,.earlyExitRange=3},false);
	
	chassis.turnToPoint(-44.03,-15.85,1000,{.forwards=false},false);
	lift.move_absolute(0,500);;
	chassis.moveToPoint(-43.03,-15.85,1000,{.forwards=false,.maxSpeed=65,.minSpeed=15,.earlyExitRange=10},false);
	chassis.moveToPoint(-43.03,-15.85,2000,{.forwards=false,.maxSpeed=30},true);
	x=0;
	while(clampDistance.get_distance()>50.8&&x<100){
		pros::delay(10);
		//master.rumble(".");
		x++;
	}
	clampPiston.set_value(false);
	pros::delay(200);
	liftTaskTarget=360;
	chassis.cancelMotion();
	chassis.turnToPoint(-18,-14.8,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=30},false);
	chassis.moveToPoint(-18,-14.8,1000,{.forwards=false,.maxSpeed=82,.minSpeed=15,.earlyExitRange=15},false);
	chassis.moveToPoint(-18,-14.8,300,{.forwards=false,.maxSpeed=50},false);
	liftTaskTarget=270;
	pros::delay(100);
	clampPiston.set_value(true);
	pros::delay(200);
	chassis.moveToPoint(-32,-12,1000,{},false);
	lift.move_absolute(0,500);;
	chassis.turnToPoint(-18,8,1000,{.forwards=false},false);
	clampPiston.set_value(false);
	chassis.moveToPoint(-18,8,1000,{.forwards=false,.maxSpeed=127,.minSpeed=30,.earlyExitRange=15},false);
	chassis.moveToPoint(-18,8,1000,{.forwards=false,.maxSpeed=60,.minSpeed=60,.earlyExitRange=13},false);
	clampPiston.set_value(true);
	//chassis.moveToPoint(-18,8,2000,{.forwards=false,.maxSpeed=120,.minSpeed=70},true);
	left_motor_group.move_voltage(-12000);
	while(clampDistance.get_distance()>50.8){
		pros::delay(10);
		master.rumble(".");
	}
	clampPiston.set_value(false);
	left_motor_group.move_voltage(0);
	pros::delay(200);
	liftTaskTarget=560;
	left_motor_group.move_voltage(12000);
	right_motor_group.move_voltage(-6000);
	pros::delay(700);
	
	chassis.turnToPoint(-18,-14.8,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=20},false);
	chassis.moveToPoint(-18,-14.8,500,{.forwards=false},false);
	/*clampPiston.set_value(true);
	liftTaskTarget=710;*/
	//liftTaskTarget=450;
}

void skills(){
	
	lift.tare_position_all();


	//pros::delay(1000);
	//master.rumble("-");
	//liftTaskTarget=55;
	liftTaskTarget=150;
	roller.move_voltage(12000);
	while(!checkRed(rollerOpticalRight.get_hue())){
		pros::delay(10);
	}
	pros::delay(100);//if red
	
	roller.move_voltage(0);
	chassis.moveToPoint(5,14.8,1000,{.forwards=false,.minSpeed=10,.earlyExitRange=7},false);
	chassis.turnToPoint(16.43,13.606,1000,{.forwards=false,.minSpeed=70,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,13.606,1200,{.forwards=false,.maxSpeed=50},true);
	pros::delay(700);
	
	liftTaskTarget=90;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(300);
	goalReset(16.43,13.606);
	
	chassis.moveToPoint(11,26,2000,{},false);
	lift.move_absolute(0,400);
	chassis.turnToPoint(16,37.4,1000,{.forwards=false},false);
	chassis.moveToPoint(16,37.4,1000,{.forwards=false,.maxSpeed=65,.minSpeed=15,.earlyExitRange=10},false);
	
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
	chassis.turnToPoint(16.43,13.606,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=5},false);
	chassis.moveToPoint(16.43,13.606,1000,{.forwards=false,.maxSpeed=82,.minSpeed=40,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,13.606,600,{.forwards=false,.maxSpeed=50},true);
	pros::delay(300);
	liftTaskTarget=85;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(100);
	goalReset(16.43,13.606);
	
	//chassis.moveToPoint(34,36,1500,{},false);
	chassis.turnToPoint(29,26,1000,{.maxSpeed=100,.minSpeed=50,.earlyExitRange=5},false);
	chassis.moveToPoint(29,26,1000,{.maxSpeed=100,.minSpeed=15,.earlyExitRange=3},false);
	
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
	chassis.moveToPoint(16.43,13.606,1000,{.forwards=false,.maxSpeed=82,.minSpeed=15,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,13.606,600,{.forwards=false,.maxSpeed=50},true);
	pros::delay(300);
	lift.move_absolute(0,400);
	pros::delay(100);
	clampPiston.set_value(true);
	pros::delay(200);//53.3, 15.8     54,1.6
	
	goalReset(16.43,13.606);


	double liftCup[4] = {550,740,930,0};
	int i =0;
	while(i!=3){
	chassis.moveToPoint(51,12,2000,{},false);
	//lift.move_absolute(0,400);
	chassis.turnToPoint(49.44,-2, 1000,{.forwards=false},false);
	chassis.moveToPoint(55,-2, 2000,{.forwards=false,.maxSpeed=85,.earlyExitRange=10},false);
	chassis.moveToPoint(55,-2, 2000,{.forwards=false,.maxSpeed=50},true);
	x=0;
	while(clampDistance.get_distance()>50.8&&x<300){
		pros::delay(10);
		//master.rumble(".");
		x++;
	}
	clampPiston.set_value(false);
	pros::delay(300);
	chassis.cancelMotion();
	/*chassis.moveToPoint(55,12, 100,{},false);
	chassis.moveToPoint(56, 3.5, 200, {.forwards=false},false);*/
	//pros::delay(200);
	///chassis.cancelMotion();
	//chassis.swingToPoint(40,12,DriveSide::LEFT,1000,{.minSpeed=30,.earlyExitRange=20},false);
	//	liftTaskTarget=liftCup[i];
	//chassis.moveToPoint(40,12,2000,{/*.minSpeed=10,.earlyExitRange=15*/},false);

	//chassis.turnToPoint(18,12,2000,{.forwards=false/*,.minSpeed=50,.earlyExitRange=30*/},false);
	//chassis.moveToPoint(18,12,1000,{.forwards=false,.maxSpeed=50},false);
	chassis.moveToPoint(56,12,1000,{},false);
	liftTaskTarget=liftCup[i];
	chassis.turnToPoint(16.43,13.606,1000,{.forwards=false},false);
	chassis.moveToPoint(16.43,13.606,1000,{.forwards=false,.maxSpeed=70,.minSpeed=15,.earlyExitRange=15},false);
	chassis.moveToPoint(16.43,13.606,900,{.forwards=false,.maxSpeed=50},true);
	pros::delay(500);
	lift.move_absolute(0,400);//liftTaskTarget=liftCup[i]100;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);
	goalReset(16.43,13.606);
	i++;
 	}

	chassis.moveToPoint(53,15.8,2000,{},false);
	lift.move_absolute(-0.2,400);
	chassis.turnToPoint(55,-5, 1000,{.forwards=false},false);
	chassis.moveToPoint(55,-5, 1000,{.forwards=false,.maxSpeed=70,.minSpeed=10,.earlyExitRange=12},false);
	chassis.moveToPoint(55,-5, 500,{.forwards=false,.maxSpeed=60},false);
	clampPiston.set_value(false);//42.2,35.5
	pros::delay(400);

	//lift.move_absolute(0.75,600);
	chassis.moveToPoint(54,10,300,{.forwards=false},false);
	//pros::delay(200);
	chassis.swingToPoint(44,15,DriveSide::LEFT,1000,{.minSpeed=15,.earlyExitRange=15},false);
	chassis.moveToPoint(44,15,2000,{},false);
	liftTaskTarget=150;
	chassis.turnToPoint(44,39,2000,{.forwards=false,},false);
	chassis.moveToPoint(44,39,1000,{.forwards=false,.maxSpeed=50},false);
	liftTaskTarget=80;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);
	liftTaskTarget=90;
	pros::delay(400);
	clampPiston.set_value(false);
	pros::delay(200);
	liftTaskTarget=200;


	liftCup[0] = 175;
	liftCup[1]= 360;
	liftCup[2] = 550;
	liftCup[3] = 740;
	i=0;
	while(i<2){
	
	
	
	if(i==0){
		chassis.moveToPoint(42.2,22,1000,{},false);
		chassis.turnToHeading(90,1000,{},false);
		clampPiston.set_value(true);
		pros::delay(200);
		lift.move_absolute(0,400);
		chassis.moveToPoint(63,22,1500,{.maxSpeed=80},false);
		chassis.setPose(0,chassis.getPose().y,270);
		chassis.moveToPoint(-6,22,1000,{.forwards=false},false);
		chassis.turnToPoint(-6,12,1000,{});

	}
		
		chassis.swingToPoint(-6,12,DriveSide::LEFT,1000,{.minSpeed=15,.earlyExitRange=15},false);
		//lift.move_absolute(0,400);
		chassis.moveToPoint(-6,12,2000,{},false);
		chassis.turnToPoint(-6,-6, 2000,{.forwards=false,.maxSpeed=70},false);
	
	chassis.waitUntilDone();
	chassis.moveToPoint(-6,-6, 2000,{.forwards=false},true);
	x=0;
	while(clampDistance.get_distance()>50.8&&x<300){
		pros::delay(10);
		master.rumble(".");
		x++;
	}
	clampPiston.set_value(false);
	chassis.moveToPoint(-6,12, 100,{.forwards=false},false);
	chassis.moveToPoint(-6, -5, 200, {.forwards=false},false);
	
	pros::delay(200);
	chassis.cancelMotion();
	chassis.swingToPoint(-17,15,DriveSide::LEFT,1000,{.minSpeed=15,.earlyExitRange=15},false);
	chassis.moveToPoint(-17,15,2000,{},false);
	liftTaskTarget=liftCup[i];
	chassis.turnToPoint(-17,39,2000,{.forwards=false},false);
	chassis.moveToPoint(-17,39,1000,{.forwards=false,.maxSpeed=50},false);
	lift.move_absolute(0,400);//liftTaskTarget=liftCup[i]-100;
	pros::delay(200);
	clampPiston.set_value(true);
	pros::delay(200);
	
	i++;

	}
	chassis.turnToPoint(-35,22,2000,{.minSpeed=70,.earlyExitRange=15},false);
	chassis.moveToPoint(-35,22,2000,{.minSpeed=10,.earlyExitRange=5},false);
	chassis.turnToPoint(-34,61,2000,{.forwards=false,.minSpeed=50,.earlyExitRange=20},false);
	chassis.moveToPoint(-34,61,3000,{.forwards=false},false);
	chassis.turnToPoint(1,63,2000,{.minSpeed=70,.earlyExitRange=15},false);
	chassis.moveToPoint(1,63,1200,{},false);
	chassis.setBrakeMode(MOTOR_BRAKE_HOLD);

	roller.move_voltage(12000);
	while(!checkRed(rollerOpticalRight.get_hue())){
		pros::delay(10);
	}
	//pros::delay(200);//if red
	
	roller.move_voltage(0);
	chassis.moveToPoint(-50,63,2500,{.forwards=false},false);
	

	
	

	
	

	/*chassis.turnToHeading(45, 1000,{},false);
	chassis.turnToHeading(135, 1000,{},false);
	chassis.turnToHeading(315, 1000,{},false);*/
	//chassis.moveToPoint(0,48,4000,{},false);
	//chassis.moveToPoint(0,0,4000,{.forwards=false},false);
}
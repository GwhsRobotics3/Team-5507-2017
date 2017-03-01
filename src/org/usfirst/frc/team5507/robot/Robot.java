package org.usfirst.frc.team5507.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Check out all our code. You won't understand it. 
 * Master He wrote the first half, Legina Chen wrote the																															I should be the new software leader.
 * second half, but Meau Bonton is the "real" software leader.
 * Call (415) 400-6078 for help. Good luck.
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	SendableChooser autoChooser;
	Joystick stick = new Joystick(0); 
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);
	Spark climber = new Spark(4);
	Timer timer = new Timer();
	Timer tAcceleration = new Timer();
	Timer timerMotor = new Timer();
	Timer timerAuto = new Timer();
	Timer timerPneumatics = new Timer();
	double maxAcceleration = 0.;
	Compressor c = new Compressor(0);
	int caseAuto = 0;
	int state = 0;
	int autonomousState = 0;
	DigitalOutput relay = new DigitalOutput(0);
	DoubleSolenoid solenoid1 = new DoubleSolenoid(0, 1);
	DoubleSolenoid solenoid2 = new DoubleSolenoid(2, 3);
	static Camera camera;
	private final Object imgLock = new Object();
	//Tony broke the robot(that's right, again...)
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code. B Money is funny :)
	 */
	@Override
	public void robotInit() {
		//pipeline = new GripPipeline();
		camera = new Camera();
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Box", 0);
		autoChooser.addObject("Camera", 1);
		autoChooser.addObject("Straight", 2);
		autoChooser.addObject("Camera Left", 3);
		autoChooser.addObject("Camera Right", 4);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
		myRobot.setInvertedMotor(MotorType.kRearLeft, false);
		myRobot.setInvertedMotor(MotorType.kFrontLeft, false);
		myRobot.setInvertedMotor(MotorType.kRearRight, true);
		myRobot.setInvertedMotor(MotorType.kFrontRight, true);
		c.setClosedLoopControl(true);
	}
	
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		this.closeGearHolder();
		timer.reset();
		timer.start();
		timerAuto.start();
		caseAuto = (int) autoChooser.getSelected();
		autonomousState = 0;
		c.setClosedLoopControl(true);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("autoTimer", timerAuto.get());
		SmartDashboard.putNumber("caseAuto", caseAuto);
		
		switch(caseAuto){
		case 0:
			this.autonomousBox();
			break;
		case 1:
			this.autonomousCamera();
			break;
		case 2:
			this.autonomousDriveStraight();
			break;
		case 3:
			this.autonomousCameraLeftSide();
			break;
		case 4:
			this.autonomousCameraRightSide();
			break;
		}
				//myRobot.mecanumDrive_Cartesian(1, -0.055, 0, 0); //right
				//myRobot.mecanumDrive_Cartesian(-1, 0.07, -0.1025, 0);// left
				//myRobot.mecanumDrive_Cartesian(0.15, 1, 0, 0); //back
				//myRobot.mecanumDrive_Cartesian(0.09, -1, 0, 0); //forward
	}
	
	/**
	 * method to get horizontal offset from center
	 * @return a double representing horizontal offset from center
	 */
	public double getCameraHorizontal() { //image is rotated 270 degrees because camera is placed sideways
		double cameraOffsetFromCenter = 0;
		double temp_centerY = camera.getCenterY();
		double temp_imgHeight = (double)camera.getImgHeight();
		double temp_numerator = temp_centerY-temp_imgHeight/2.0;
		cameraOffsetFromCenter = temp_numerator/(temp_imgHeight/2.0);
		return cameraOffsetFromCenter;
	}
	
	/**
	 * method to get vertical offset from center
	 * @return a double representing vertical offset from center
	 */
	public double getCameraVertical() {
		double cameraOffsetFromCenter = 0;
		double temp_centerX = camera.getCenterX();
		double temp_imgWidth = (double)camera.getImgWidth();
		double temp_numerator = temp_centerX-temp_imgWidth/2.0;
		cameraOffsetFromCenter = temp_numerator/(temp_imgWidth/2.0);
		return cameraOffsetFromCenter;
	}
	
	/**
	 *This is code for the "Drive Straight" option in autonomous code.
	 */
	public void autonomousDriveStraight() {
		switch (autonomousState){
		case 0:
			myRobot.mecanumDrive_Cartesian(0.09, -1, 0, 0);
			if(timerAuto.get() > 0.5){
				autonomousState = 1;
			}
			break;
		case 1:
			myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
			break;
		}
		
	}
	
	/**
	 * method to get approx. distance between wall and robot in inches
	 * @param pixels number of pixels
	 * @return a double representing inches from wall and robot 
	 */
	public double getDistanceToWallInInches(double pixels) {
		return 0.0039494*pixels*pixels - 2.3615*pixels + 361.92;
	}
	
	/**
	 * Code that gets a value from -1 to 1 representing x value distance from center.
	 * @return value from -1 to 1 representing offset from center
	 */
	public double getCameraOffsetFromCenter() { //image is rotated 270 degrees
		return this.getCameraHorizontal();
	}
	
	/**
	 * Use this autonomous mode when starting from the center
	 */
	public void autonomousCamera() {
		SmartDashboard.putNumber("Distance from wall: ", this.getDistanceToWallInInches(camera.getCenterX()));
		SmartDashboard.putNumber("Autonomous State: ", autonomousState);
		double xDrive = 0;
		double yDrive = 0;
		double rotateDrive = 0;
		
		switch(autonomousState) {
		case 0: //drive forward until we get to the gear peg
			if (this.getCameraOffsetFromCenter()<-0.12) {
				xDrive = -0.4;
			}
			else if (this.getCameraOffsetFromCenter()>0.12) {
				xDrive = 0.4; //right
			}
			yDrive = -0.25; //forward
			if (this.getDistanceToWallInInches(camera.getCenterX()) < 12) {
				autonomousState = 1;
				timerAuto.reset();
			}
			break;
		case 1:
			xDrive = this.getCameraOffsetFromCenter()*(0.25);
			yDrive = -0.25;//forward
			if(timerAuto.get() > 1.0){
				autonomousState = 2;
				timerAuto.reset();
			}
			break;
		case 2: //open gear holder and wait
			this.openGearHolder();
			if(timerAuto.get() > 1.5){
				autonomousState =3;
				timerAuto.reset();
			}
			break;
		case 3: //drive back to clear
			xDrive = 0.15*0.25;
			yDrive = 0.25;//back
			if(timerAuto.get() > 3.0){
				timerAuto.reset();
				autonomousState = 4;
			}
			break;
		case 4: //drive back to clear
			xDrive = 0.15*0.25;
			yDrive = 0.25;//back
			if(timerAuto.get() > 1.0){
				timerAuto.reset();
				autonomousState = 5;
			}
			break;
		case 5:
			this.closeGearHolder();
			//xDrive = 1*0.25;
			//yDrive = -0.055*0.25;//right
			xDrive = 0;
			yDrive = 0;
			rotateDrive = 1;
			//yDrive = -0.5;
			if(timerAuto.get() > 0.2){
				autonomousState = 6;
			}
			break;
		case 6:
			yDrive = -0.5;
			if(timerAuto.get() > 0.25){
				autonomousState = 7;
			}
		case 7:
			xDrive = 0;
			yDrive = 0;
			rotateDrive = -1;
			//yDrive = -0.5;
			if(timerAuto.get() > 0.2){
				autonomousState = 8;
			}
			break;
		case 8:
			xDrive = 0;
			yDrive = -0.5;
			if(timerAuto.get() > 0.25){
				autonomousState = 9;
			}
			break;
		case 9:
			xDrive = this.getCameraOffsetFromCenter()*0.25;
			yDrive = -1*0.25; //forward
			if(timerAuto.get() > 4.0){
				autonomousState = 10;
			}
			break;
		case 10:
			break;
		
		}
		SmartDashboard.putNumber("xDrive: ", xDrive);
		SmartDashboard.putNumber("yDrive: ", yDrive);
		SmartDashboard.putNumber("rotateDrive: ", rotateDrive);
		myRobot.mecanumDrive_Cartesian(xDrive, yDrive, rotateDrive, 0);
	}
	
	/**
	 * method to autonomously drive robot if it starts on the left side 
	 */
	public void autonomousCameraLeftSide() {
		SmartDashboard.putNumber("Distance from wall: ", this.getDistanceToWallInInches(camera.getCenterX()));
		SmartDashboard.putNumber("Autonomous State: ", autonomousState);
		double xDrive = 0;
		double yDrive = 0;
		double rotateDrive = 0;
		
		switch(autonomousState) {
		case 0:
			xDrive = -0.5;
			yDrive = 0;
			rotateDrive = -0.025;
			
			if(camera.getContoursFound() > 1 && this.getCameraHorizontal() < 0.1 && this.getCameraHorizontal() > -0.1){
				autonomousState = 1;
				timerAuto.reset();
			}
			break;
		case 1: //drive forward until we get to the gear peg
			if(this.getCameraOffsetFromCenter() < -0.05){
				xDrive = -0.4;
			}
			else if(this.getCameraOffsetFromCenter() > 0.05){
				xDrive = 0.4; //right
			}
			yDrive = -0.25; //forward
			if(this.getDistanceToWallInInches(camera.getCenterY()) < 12){
				autonomousState = 2;
				timerAuto.reset();
			}
			break;
		case 2:
			xDrive = this.getCameraOffsetFromCenter()*(0.25);
			yDrive = -0.25;//forward
			if(timerAuto.get() > 1.0){
				autonomousState = 3;
				timerAuto.reset();
			}
			break;
		case 3: //open gear holder and wait
			this.openGearHolder();
			if(timerAuto.get() > 1.5){
				autonomousState =4;
				timerAuto.reset();
			}
			break;
		case 4: //drive back to clear
			xDrive = 0.15*0.25;
			yDrive = 0.25;//back
			if(timerAuto.get() > 3.0){
				timerAuto.reset();
				autonomousState = 5;
			}
			break;
		case 5:
			this.closeGearHolder();
			//xDrive = 1*0.25;
			//yDrive = -0.055*0.25;//right
			xDrive = 0;
			yDrive = 0;
			rotateDrive = -1;
			//yDrive = -0.5;
			if(timerAuto.get() > 0.5){
				autonomousState = 6;
			}
			break;
		case 6:
			yDrive = -0.5;
			if(timerAuto.get() > 0.25){
				autonomousState = 7;
			}
		case 7:
			xDrive = 0;
			yDrive = 0;
			rotateDrive = 0.5;
			//yDrive = -0.5;
			if(timerAuto.get() > 0.5){
				autonomousState = 8;
			}
			break;
		case 8:
			xDrive = 0;
			yDrive = -0.5;
			if(timerAuto.get() > 0.25){
				autonomousState = 9;
			}
			break;
		case 9:
			xDrive = this.getCameraOffsetFromCenter()*0.25;
			yDrive = -1*0.25; //forward
			if(timerAuto.get() > 4.0){
				autonomousState = 10;
			}
			break;
		case 10:
			break;
		}
		SmartDashboard.putNumber("xDrive: ", xDrive);
		SmartDashboard.putNumber("yDrive: ", yDrive);
		SmartDashboard.putNumber("rotateDrive: ", rotateDrive);
		myRobot.mecanumDrive_Cartesian(xDrive, yDrive, rotateDrive, 0);
	}
	
	/**
	 * method to autonomously drive robot if it starts on the right side
	 */
	public void autonomousCameraRightSide() {
		SmartDashboard.putNumber("Distance from wall: ", this.getDistanceToWallInInches(camera.getCenterX()));
		SmartDashboard.putNumber("Autonomous State: ", autonomousState);
		double xDrive = 0;
		double yDrive = 0;
		double rotateDrive = 0;
		
		switch(autonomousState){
		case 0:
			xDrive = 0.5;
			yDrive = 0;
			rotateDrive = 0.025;
			
			if(camera.getContoursFound() > 1 && (this.getCameraHorizontal()<0.1 && this.getCameraHorizontal()>-0.1)){
				autonomousState = 1;
				timerAuto.reset();
			}
			break;
		case 1: //drive forward until we get to the gear peg
			if(this.getCameraOffsetFromCenter()<-0.05){
				xDrive = 0.4;
			}
			else if(this.getCameraOffsetFromCenter()>0.05){
				xDrive = -0.4; //right
			}
			yDrive = -0.25; //forward
			if(this.getDistanceToWallInInches(camera.getCenterX()) < 12){
				autonomousState = 2;
				timerAuto.reset();
			}
			break;
		case 2:
			xDrive = this.getCameraOffsetFromCenter()*(0.25);
			yDrive = -0.25;//forward
			if(timerAuto.get() > 1.0){
				autonomousState = 3;
				timerAuto.reset();
			}
			break;
		case 3: //open gear holder and wait
			this.openGearHolder();
			if(timerAuto.get() > 1.5){
				autonomousState =4;
				timerAuto.reset();
			}
			break;
		case 4: //drive back to clear
			xDrive = 0.15*0.25;
			yDrive = 0.25;//back
			if(timerAuto.get() > 3.0){
				timerAuto.reset();
				autonomousState = 5;
			}
			break;
		case 5:
			this.closeGearHolder();
			xDrive = 1*0.25;
			yDrive = -0.055*0.25;//right
			if(timerAuto.get() > 0.5){
				autonomousState = 6;
			}
			break;
		case 6:
			xDrive = this.getCameraOffsetFromCenter()*0.25;
			yDrive = -1*0.25; //forward
			if(timerAuto.get() > 4.0){
				autonomousState = 7;
			}
			break;
		case 7:
			break;
		}
		SmartDashboard.putNumber("xDrive: ", xDrive);
		SmartDashboard.putNumber("yDrive: ", yDrive);
		SmartDashboard.putNumber("rotateDrive: ", rotateDrive);
		myRobot.mecanumDrive_Cartesian(xDrive, yDrive, rotateDrive, 0);
	}
		
	/**
	 * method to autonomously drive robot in a box
	 */
	public void autonomousBox() {
		if(timerAuto.get()>0 && timerAuto.get()<0.25){
			myRobot.mecanumDrive_Cartesian(0.09, -1, 0, 0); //FORWARD
			}
		if(timerAuto.get()>0.5 && timerAuto.get()<2.5){
			myRobot.mecanumDrive_Cartesian(0.0, 0, 0, 0);
		}
		if(timerAuto.get()>2.5 && timerAuto.get()<3.5){
			myRobot.mecanumDrive_Cartesian(-1.0, 0.07, -0.1025, 0);// left
			}
		if(timerAuto.get()>3.5 && timerAuto.get()<5.0){
			myRobot.mecanumDrive_Cartesian(0.0, 0, 0, 0);
		}
		if(timerAuto.get()>5.0 && timerAuto.get()<6.0){
			myRobot.mecanumDrive_Cartesian(1, -0.055, 0, 0);//right
		}
		if(timerAuto.get()>6.0 && timerAuto.get()<7.5){
			myRobot.mecanumDrive_Cartesian(0.0, 0, 0, 0);
		}
		if(timerAuto.get()>7.5 && timerAuto.get()<7.75){
			myRobot.mecanumDrive_Cartesian(0.15, 1, 0, 0); //back 
		}
	}
	
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() { 
		this.openGearHolder();
	}
		
	public void closeGearHolder() {
		solenoid1.set(DoubleSolenoid.Value.kReverse);
		solenoid2.set(DoubleSolenoid.Value.kReverse);
		SmartDashboard.putString("Gear holder: ", "Closed");
	}
	
	public void openGearHolder() {
		solenoid1.set(DoubleSolenoid.Value.kForward);
		solenoid2.set(DoubleSolenoid.Value.kForward);
		SmartDashboard.putString("Gear holder: ", "Opened");
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
//		if(stick.getRawButton(3)){
//			relay.set(true);
//		}
//		if(stick.getRawButton(4)){
//			relay.set(false);
//		}
		double xDrive = 0.;
		double yDrive = 0.;
		double rotateDrive = 0.0;
		
		if (Math.abs(stick.getX()) > 0.25 ) {
			tAcceleration.reset();
			tAcceleration.start();
			c.stop(); //Need to test to stop compressor
//			if(tAcceleration.get() > 0 && tAcceleration.get() < 0.5){ //Acceleration code
//				maxAcceleration = 0.25;
//			}
//			if(tAcceleration.get() > 0.5 && tAcceleration.get() < 1.){
//				maxAcceleration = 0.5;
//			}
//			if(tAcceleration.get() > 1. && tAcceleration.get() < 1.5){
//				maxAcceleration = 0.75;
//			}
//			if(tAcceleration.get() > 1.5 && tAcceleration.get() < 2){
//				maxAcceleration = 1.;
//			}
			xDrive = stick.getX()*(Math.min(1., tAcceleration.get()/2.0));
		}
		else {
			c.start(); //Need to test to start again
		}
		if (Math.abs(stick.getY()) > 0.25 ) {
			yDrive = stick.getY();
		}
		if ((Math.abs(stick.getRawAxis(2) - stick.getRawAxis(3)) > 0.1 )) {
			rotateDrive = stick.getRawAxis(2) - stick.getRawAxis(3);
		}
		
		//computer assisted line-up
		if (stick.getRawButton(6))
		{
			if (this.getCameraOffsetFromCenter() < -0.05) {
				xDrive = -0.4;
			}
			else if (this.getCameraOffsetFromCenter() > 0.05) {
				xDrive = 0.4; //right
			}
			SmartDashboard.putString("Assist Mode: ", "on");
		}
		else {
			SmartDashboard.putString("Assist Mode: ", "off");
		}
		
		myRobot.mecanumDrive_Cartesian(xDrive, yDrive, -rotateDrive,0); //0.75 ceiling on max speed
		
		
		SmartDashboard.putNumber("xDrive: ", xDrive);
		SmartDashboard.putNumber("yDrive: ", yDrive);
		SmartDashboard.putNumber("rotateDrive: ", rotateDrive);	
		SmartDashboard.putNumber("centerX: ", camera.getCenterY());
		SmartDashboard.putNumber("centerY: ", camera.getCenterX());
		SmartDashboard.putNumber("imgHeight: ", camera.getImgHeight());

		// control Climber code
		if (stick.getRawButton(1)) {// Button 1 is A
			climber.set(1);
			SmartDashboard.putString("Climber Direction : ", "down");
		}
		else if (stick.getRawButton(2) ) { // button 2 is B 
			climber.set(-1);
			SmartDashboard.putString("Climber Direction : ", "up");
		}
		else {
			climber.set(0);
		}
		
		if (timerPneumatics.get() == 0){
			timerPneumatics.start();
		}
		
		switch(state) {
			case 0 : // Open Listen for Button
				openGearHolder();
				if(stick.getRawButton(5)){
					state = 1;
					timerPneumatics.reset();
				}
				break;
				
			case 1 : // Closed Wait for Timer
				closeGearHolder();
				if(timerPneumatics.get() > 0.5){
					state = 2;
				}
				break;
				
			case 2 : // Closed Listen For Button
				closeGearHolder();
				if(stick.getRawButton(5)){
					state = 3;
					timerPneumatics.reset();
				}
				break;
				
			case 3 : // Open Wait for Timer
				openGearHolder();
				if(timerPneumatics.get() > 0.5){
					state = 0;
				}
				break;
				
			default :
				state = 0;
				break;
		}
	}
		 
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	/*
	 * Pixels    |    Distance from Goal
	 * 191            58 
	 * 197            49
	 * 210            37
	 * 235            25
	 * 286            13
	 * 307            7
	 */
}

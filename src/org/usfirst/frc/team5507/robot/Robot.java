package org.usfirst.frc.team5507.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.command.Command; //may delete
//import edu.wpi.first.wpilibj.buttons.Button; //may delete
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * Check out the camera class. You won't understand it. 
 * Howard He wrote the first half, Legina Chen wrote the
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
	Timer timerMotor = new Timer();
	Timer timerAuto = new Timer();
	Timer timerPneumatics = new Timer();
	
	Compressor c = new Compressor(0);
	
	int caseAuto = 0;
	int state = 0;
	int autonomousState = 0;
	
	DigitalOutput relay = new DigitalOutput(0);
	
	DoubleSolenoid solenoid1 = new DoubleSolenoid(0, 1);
	DoubleSolenoid solenoid2 = new DoubleSolenoid(2, 3);
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private GripPipeline pipeline = new GripPipeline();
	static Camera camera;
	private final Object imgLock = new Object();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		this.closeGearHolder();
		camera = new Camera();
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Box", 0);
		autoChooser.addObject("Camera", 1);
		autoChooser.addObject("Straight", 2);
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
		timer.reset();
		timer.start();
		timerAuto.start();
		caseAuto = (int) autoChooser.getSelected();	
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
		case 1:
			this.autonomousCamera();
		case 2:
			this.autonomousDriveStraight();
		}
				//myRobot.mecanumDrive_Cartesian(1, -0.055, 0, 0); //right
				//myRobot.mecanumDrive_Cartesian(-1, 0.07, -0.1025, 0);// left
				//myRobot.mecanumDrive_Cartesian(0.15, 1, 0, 0); //back
				//myRobot.mecanumDrive_Cartesian(0.09, -1, 0, 0); //forward
	}
	
	/**
	 *This is code for the "Drive Straight" option in autonomous code.
	 */
	public void autonomousDriveStraight(){
		switch (autonomousState){
		case 0:
			myRobot.mecanumDrive_Cartesian(0.09, -1, 0, 0);
			if(timerAuto.get() > 3.0){
				autonomousState = 1;
			}
		case 1:
			myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
		}
	}
	
	/**
	 * Code that gets a value from -1 to 1 representing x value distance from center.
	 * @return value from -1 to 1 representing offset from center
	 */
	public double getCameraOffsetFromCenter(){ //image is rotated 270 degrees
		double cameraOffsetFromCenter = 0;
		if(camera.getCenterY() != 0){
			double temp_centerY = camera.getCenterY();
			double temp_imgHeight = (double)camera.getImgHeight();
			double temp_numerator = temp_centerY-temp_imgHeight/2.0;
			cameraOffsetFromCenter = temp_numerator/(temp_imgHeight/2.0);
			//cameraOffsetFromCenter = ((camera.getCenterY()-(double)(camera.getImgHeight())/2.0))/(double)(camera.getImgHeight())/2.0;
		}
		return cameraOffsetFromCenter;
	}
	
	/**
	 * Use this autonomous mode when starting from the center
	 */
	public void autonomousCamera(){
		
		
		switch(autonomousState){
		case 0: //drive forward until we get to the gear peg
			myRobot.mecanumDrive_Cartesian(this.getCameraOffsetFromCenter(), -1, 0, 0); //forward
			if(camera.getCenterX() == 0){
				autonomousState = 1;
				timerAuto.reset();
			}
		case 1: //open gear holder and wait
			this.openGearHolder();
			if(timerAuto.get() > 3.0){
				autonomousState =2;
				timerAuto.reset();
			}
		case 2: //drive back to clear
			myRobot.mecanumDrive_Cartesian(0.15, 1, 0, 0); //back
			this.closeGearHolder();
			if(timerAuto.get() > 0.5){
				timerAuto.reset();
				autonomousState = 3;
			}
		case 3:
			myRobot.mecanumDrive_Cartesian(1, -0.055, 0, 0); //right
			if(timerAuto.get() > 0.5){
				autonomousState = 4;
			}
		case 4:
			myRobot.mecanumDrive_Cartesian(this.getCameraOffsetFromCenter(), -1, 0, 0); //forward
			if(timerAuto.get() > 4.0){
				autonomousState = 5;
			}
		case 5:
			myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
		}
	}
		
	public void autonomousBox(){
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
	}
		
	public void closeGearHolder(){
		solenoid1.set(DoubleSolenoid.Value.kReverse);
		solenoid2.set(DoubleSolenoid.Value.kReverse);
		SmartDashboard.putString("Gear holder: ", "Closed");
	}
	
	public void openGearHolder(){
		solenoid1.set(DoubleSolenoid.Value.kForward);
		solenoid2.set(DoubleSolenoid.Value.kForward);
		SmartDashboard.putString("Gear holder: ", "Open");
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
		
		if(Math.abs(stick.getX())>0.25 ) {
			xDrive = stick.getX();
		}
		if(Math.abs(stick.getY())>0.25 ) {
			yDrive = stick.getY();
		}
		if((Math.abs(stick.getRawAxis(2)- stick.getRawAxis(3))>0.1 )) {
			rotateDrive = stick.getRawAxis(2) - stick.getRawAxis(3);
		}
		
		//computer assisted line-up
		if(stick.getRawButton(6))
		{
			xDrive = this.getCameraOffsetFromCenter();
			SmartDashboard.putString("Assist Mode: ", "on");
			myRobot.mecanumDrive_Cartesian(xDrive, yDrive, -rotateDrive,0);
		}
		else{
			SmartDashboard.putString("Assist Mode: ", "off");
		}
		
		myRobot.mecanumDrive_Cartesian(xDrive, yDrive, -rotateDrive,0);
		
		SmartDashboard.putNumber("xDrive: ", xDrive);
		SmartDashboard.putNumber("yDrive: ", yDrive);
		SmartDashboard.putNumber("rotateDrive: ", rotateDrive);	
		SmartDashboard.putNumber("centerX: ", camera.getCenterY());
		SmartDashboard.putNumber("centerY: ", camera.getCenterX());
		SmartDashboard.putNumber("imgHeight: ", camera.getImgHeight());

		// control Climber code
		if(stick.getRawButton(1)) {// Button 1 is A
			climber.set(1);
			SmartDashboard.putString("Climber Direction : ", "down");
		} 
		else if(stick.getRawButton(2) ) { // button 2 is B 
			climber.set(-1);
			SmartDashboard.putString("Climber Direction : ", "up");
		} 
		else{
			climber.set(0);
		}
		
		if(timerPneumatics.get() == 0){
			timerPneumatics.start();
		}
		
		
		switch(state){
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
}

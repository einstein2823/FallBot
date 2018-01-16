/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	public class Button {
		private boolean h = false; // stands for 'held', true if the Button is being actively held down
		private boolean s = false; // stands for 'state', true if the Button is pressed
		private boolean ls = false; // stands for 'last state', stores the previous state of the Button
		private boolean c = false; // stands for 'changed', true if the Button's previous state does not match its
		// current state

		// check if the Button is pressed
		public boolean on() {
			return s;
		}

		// check if the Button is held down
		public boolean held() {
			return h;
		}

		// check if the Button has changed
		public boolean changed() {
			return c;
		}

		// update the Button, should be called periodically
		public void update(boolean b) {

			if (b && (b != ls)) {
				s = !s;
				c = true;

			} else {
				c = false;
			}

			h = b;
			ls = b;
		}

		// reset all values
		public void reset() {
			s = false;
			ls = false;
			c = false;
		}

	}

	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	Joystick joystick;

	Button buttonA;
	Button buttonB;
	Button buttonX;

	TalonSRX motor0;
	TalonSRX motor1;
	TalonSRX motor2;
	TalonSRX motor3;
	TalonSRX motor4;
	TalonSRX motor5;

	DoubleSolenoid solenoid1;
	Compressor compressor;
	Encoder leftEncoder;
	Encoder rightEncoder;
	
	SnazzyMotionPlanner leftControl;
	SnazzyMotionPlanner rightControl;
	
	LeftDrivePIDOutput lDriveOutput;
	RightDrivePIDOutput rDriveOutput;
	
	
	double maxMotorPower = .8; // maximum motor power output for 775's - owen said 0.8, but trying 
	double motorRampRate = 36;// owen
	//double lastShift;
	
	//final double ENC_TO_IN = 2 * Math.PI * WHEEL_RADIUS * DRIVE_RATIO / ENCODER_RESOLUTION;
	//final double IN_TO_ENC = ENCODER_RESOLUTION / (2 * Math.PI * WHEEL_RADIUS * DRIVE_RATIO);
	
	DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kForward;
	
	double wheelDiameter = 4;
	double highGearDistancePerRev = (wheelDiameter * Math.PI) / 12.2646604938; // 12.2646604938 775 turns per wheel rev
	double lowGearDistancePerRev = (wheelDiameter * Math.PI) / 30.9375; // 30.9375 775 turns per wheel rev
	double highGearDistancePerPulse = (highGearDistancePerRev / 12)*((15+5/8)/4); // 12 pulses per rev
	double lowGearDistancePerPulse = (lowGearDistancePerRev / 12)*((15+5/8)/4); // 12 pulses per rev; ((15-5/8)/4) multiplier  because Owen probably did his math wrong
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		/** UsbCamera c = CameraServer.getInstance().startAutomaticCapture();
	        c.setResolution(320, 180);
	        c.setFPS(29);
	        **/
		joystick = new Joystick(0);

		buttonA = new Button();
		buttonB = new Button();
		buttonX = new Button();
		
		motor0 = new TalonSRX(11);
		motor1 = new TalonSRX(12);
		motor2 = new TalonSRX(13);
		motor3 = new TalonSRX(21);
		motor4 = new TalonSRX(22);
		motor5 = new TalonSRX(23);

		motor0.setVoltageRampRate(motorRampRate);
		motor1.setVoltageRampRate(motorRampRate);
		motor2.setVoltageRampRate(motorRampRate);
		
		motor3.setVoltageRampRate(motorRampRate);
		motor4.setVoltageRampRate(motorRampRate);
		motor5.setVoltageRampRate(motorRampRate);
		
		
		leftEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		leftEncoder.setDistancePerPulse(1);
		rightEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		rightEncoder.setDistancePerPulse(1);

		//leftEncoder.setSamplesToAverage(20);
		//rightEncoder.setSamplesToAverage(20);

		solenoid1 = new DoubleSolenoid(0, 1);
		compressor = new Compressor(0);

		solenoid1.set(lowGear);

 
		leftEncoder.reset();
		rightEncoder.reset();
		
		lDriveOutput = new LeftDrivePIDOutput(this);
		rDriveOutput = new RightDrivePIDOutput(this);
		
		
		leftControl = new SnazzyMotionPlanner(0.04, 0.001, 0.8, 0, 0.0017, 0.002,  leftEncoder, lDriveOutput, 0.01, "Left.csv");
		rightControl= new SnazzyMotionPlanner(0.04, 0.001, 0.8, 0, 0.0017, 0.002,  rightEncoder, rDriveOutput, 0.01,"Right.csv");
	
		
		SmartDashboard.putNumber("P", 0.01);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		SmartDashboard.putNumber("Setpoint", 0);
		SmartDashboard.putNumber("L Encoder", 0);
		SmartDashboard.putNumber("R Encoder", 0);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		//System.out.println("Auto selected: " + m_autoSelected);
		
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
		case kCustomAuto:
			// Put custom auto code here
			break;
		case kDefaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

	
		buttonA.update(joystick.getRawButton(2));
		buttonB.update(joystick.getRawButton(3));


		if (buttonA.changed()) {
			solenoid1.set(lowGear);
			System.out.println("low");
			//leftEncoder.setDistancePerPulse(lowGearDistancePerPulse);
			//rightEncoder.setDistancePerPulse(lowGearDistancePerPulse);

		}
		if (buttonB.changed()) {
			solenoid1.set(highGear);
			System.out.println("high");
			//leftEncoder.setDistancePerPulse(highGearDistancePerPulse);
			//rightEncoder.setDistancePerPulse(highGearDistancePerPulse);

			
		}
		
		SmartDashboard.putNumber("L Encoder", leftEncoder.get());
		SmartDashboard.putNumber("R Encoder", rightEncoder.get());
		//System.out.println(lowGearDistancePerPulse);
		
		
		motor0.set(Math.pow(joystick.getRawAxis(3), 1) * maxMotorPower);
		motor1.set(Math.pow(joystick.getRawAxis(3), 1) * maxMotorPower);
		motor2.set(Math.pow(joystick.getRawAxis(3), 1) * maxMotorPower);
		motor3.set(-Math.pow(joystick.getRawAxis(1), 1) * maxMotorPower);
		motor4.set(-Math.pow(joystick.getRawAxis(1),1) * maxMotorPower);
		motor5.set(-Math.pow(joystick.getRawAxis(1),1) * maxMotorPower);
		
		
		//This thing maybe
		//motorx.set(joystick.getRawAxis(3) > maxSpeed ? joystick.getRawAxis(3) : 0.8)
		//System.out.println(leftEncoder.getPeriod() + " " + rightEncoder.getPeriod());
//System.out.println(motor3.getOutputCurrent());
	}

	/*
	 * This function is called periodically during test mode.
	 */

	@Override
	public void testInit() {
		leftControl.disable();
		rightControl.disable();
	}

	@Override
	public void testPeriodic() {
		buttonX.update(joystick.getRawButton(1));
		
		//leftControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//rightControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		
		if(buttonX.on()){
			if(buttonX.changed()) {
				leftEncoder.reset();
				rightEncoder.reset();
				
				leftControl.configureGoal(SmartDashboard.getNumber("Setpoint", 0), 300, 300);
				rightControl.configureGoal(SmartDashboard.getNumber("Setpoint", 0), 300, 300);
				
				leftControl.enable();
				rightControl.enable();
				
				//leftControl.setSetpoint(20);
				//leftControl.enable();
				
				//rightControl.setSetpoint(20);
				//rightControl.enable();  
			}
			
		}else if (buttonX.changed()&& !buttonX.on()){
			leftControl.disable();
			rightControl.disable();
			
			//leftControl.stopCalibration();
			//rightControl.startCalibration();
		}
	}
	public void disabledInit() {
		//System.out.println("disabled");
	}
}

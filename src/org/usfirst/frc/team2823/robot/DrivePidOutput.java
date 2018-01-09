package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.MotorControl.CAN.TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;

public class DrivePidOutput implements PIDOutput {
	private TalonSRX motor;
	
	public DrivePidOutput(TalonSRX motor){
		this.motor = motor;
	}
	
	@Override
	public void pidWrite(double output) {
		motor.set(output);
	}
}

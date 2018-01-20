package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class RightDrivePIDOutput implements PIDOutput {
	
	Robot r;
	
	public RightDrivePIDOutput (Robot robot){
		r = robot;
	}
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		r.motor0.set(-output);
		r.motor1.set(-output);
		r.motor2.set(-output);
		
	}

}

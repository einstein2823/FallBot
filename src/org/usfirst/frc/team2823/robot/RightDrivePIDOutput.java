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
		r.motor3.set(output);
		r.motor4.set(output);
		r.motor5.set(output);
		
	}

}

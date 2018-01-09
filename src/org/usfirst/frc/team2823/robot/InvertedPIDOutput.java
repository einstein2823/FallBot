package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class InvertedPIDOutput implements PIDOutput {
	
	PIDOutput p;
	
	public InvertedPIDOutput (PIDOutput output){
		p = output;
	}
	
	@Override
	public void pidWrite(double output) {
		p.pidWrite(-1 * output);
	}

}
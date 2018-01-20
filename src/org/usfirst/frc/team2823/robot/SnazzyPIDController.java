package org.usfirst.frc.team2823.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class SnazzyPIDController extends SnazzyPIDCalculator {
	private java.util.Timer m_controlLoop;
	
	public SnazzyPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source,
             PIDOutput output, double period, String fname) {
		 super(Kp, Ki, Kd, Kf, source, output, period, fname);
		 m_controlLoop = new java.util.Timer();
		 m_controlLoop.schedule(new PIDTask(this), 0L, (long) (period * 1000));
		 
	 }
	  public void free() {
		    m_controlLoop.cancel();
		    synchronized (this) {
		      m_controlLoop = null;
		    }
		  super.free();
		  }
	 
	 private class PIDTask extends TimerTask {

		    private SnazzyPIDController m_controller;

		    public PIDTask(SnazzyPIDController snazzyPIDController) {
		      if (snazzyPIDController == null) {
		        throw new NullPointerException("Given PIDController was null");
		      }
		      m_controller = snazzyPIDController;
		    }

		    @Override
		    public void run() {
		      m_controller.calculate();
		      
		      synchronized(this) {
		    	  if(isEnabled()) {
						System.out.println("BOOM controller 1");
		    		  m_pidOutput.pidWrite(m_result);
		    	  }
		      }
		    }
		  }
}

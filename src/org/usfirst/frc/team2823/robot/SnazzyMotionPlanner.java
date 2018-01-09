package org.usfirst.frc.team2823.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;

public class SnazzyMotionPlanner extends SnazzyPIDCalculator {
	private java.util.Timer m_controlLoop;
	private boolean m_calibrating;
	private SnazzyLog m_calLog;
	private double m_calStart;
	private double m_lastCal;
	private double m_lastDist;
	private double m_lastV;
	private int m_count;
	private MotionWayPoint m_currentWaypoint;
	private boolean m_motionPlanEnabled = false;
	private boolean m_planFinished = false;
	private boolean m_dwell = false;
	private double m_invertMultiplier = 1.0;
	private double  m_maxVelocity;
	private double  m_maxAcceleration;
	private double  m_timeUntilMaxVelocity;
	private double  m_timeSpentCruising;
	private double  m_positionAtMaxVelocity;
	private double  m_positionAtEndOfCruise;
	private double  m_timeAtEndOfCruise;
	private double m_initTime;
	private double m_initPos;
	
	private double m_kA;
	private double m_kV;

	public class MotionWayPoint {
		double m_time;
		double m_position;
		double m_expectedVelocity;
		double m_expectedAcceleration;
	}

	public SnazzyMotionPlanner(double Kp, double Ki, double Kd, double Kf, double kA, double kV, PIDSource source, PIDOutput output,
			double period, String fname) {
		super(Kp, Ki, Kd, Kf, source, output, period, fname);
		m_controlLoop = new java.util.Timer();
		m_controlLoop.schedule(new PIDTask(), 0L, (long) (period * 1000));
		m_calLog = new SnazzyLog();
		m_kA = kA;
		m_kV = kV;
	}	 
	public void configureGoal(double goal, double max_v, double max_a, boolean dwell) {
		m_motionPlanEnabled = true;
		m_planFinished = false;
		m_dwell = dwell;
		m_initTime = Timer.getFPGATimestamp();
		m_initPos = m_pidInput.pidGet();

		//check if goal is negative
		if(goal < 0) {
			m_invertMultiplier = -1.0;
			goal = Math.abs(goal);
		} else {
			m_invertMultiplier = 1.0;
		}

		double midpoint = goal / 2;

		m_maxAcceleration = max_a;

		double t_until_midpoint = Math.sqrt( (midpoint * 2) / max_a);

		/* New formula:  v = at */

		double v_needed_to_get_to_midpoint = t_until_midpoint * max_a;

		/* Simple case:  we never hit max velocity */
		if (v_needed_to_get_to_midpoint <= max_v) {
			m_maxVelocity = v_needed_to_get_to_midpoint;
			m_timeUntilMaxVelocity = t_until_midpoint;
			m_timeSpentCruising = 0.0;
		} else {
			/* Complex case:  we accelerate up to max v, cruise for a while, and then decelerate */
			m_maxVelocity = max_v;

			/* v = at , so t = v/a */
			m_timeUntilMaxVelocity = max_v/max_a;

			/* d = 1/2 at^2 */
			double distance_while_accelerating = 0.5 * max_a *
					(m_timeUntilMaxVelocity * m_timeUntilMaxVelocity);

			double distance_while_cruising = goal - (2 * distance_while_accelerating);

			m_timeSpentCruising = distance_while_cruising / max_v;
		}
		m_positionAtMaxVelocity = 0.5 * m_maxAcceleration * (m_timeUntilMaxVelocity * m_timeUntilMaxVelocity);
		m_positionAtEndOfCruise = m_positionAtMaxVelocity + (m_timeSpentCruising * m_maxVelocity);
		m_timeAtEndOfCruise = m_timeUntilMaxVelocity + m_timeSpentCruising;

	}

	//overload configureGoal so that, if a dwell value is not given, it defaults to false
	public void configureGoal(double goal, double max_v, double max_a) {
		configureGoal(goal, max_v, max_a, false);
	}

	public MotionWayPoint getCurrentWaypoint(double t) {

		if (t > (2 * m_timeUntilMaxVelocity) + m_timeSpentCruising) {
			return null;
		}

		MotionWayPoint p = new MotionWayPoint();
		p.m_time = t;
		if (t < m_timeUntilMaxVelocity) {
			/* We are still ramping up.  */
			p.m_expectedVelocity = t * m_maxAcceleration;
			p.m_expectedAcceleration = m_maxAcceleration;
			p.m_position = 0.5 * m_maxAcceleration * t * t;
		} else {
			/* We are either cruising, or ramping down */
			if (t < m_timeAtEndOfCruise) {
				/* We are cruising */
				p.m_expectedVelocity = m_maxVelocity;
				p.m_expectedAcceleration = 0;
				p.m_position = m_positionAtMaxVelocity + (t - m_timeUntilMaxVelocity) * m_maxVelocity;
			} else {
				/* We are ramping down */
				double t_decel = (t - m_timeAtEndOfCruise);
				p.m_expectedAcceleration = -1.0 * m_maxAcceleration;
				p.m_expectedVelocity = m_maxVelocity - (t_decel * m_maxAcceleration);

				/* d = d0 + v0*t + 1/2 a t^2   */
				p.m_position = m_positionAtEndOfCruise + 
						(m_maxVelocity * t_decel) +
						(0.5 * -1.0 * m_maxAcceleration * (t_decel * t_decel));
			}
		}

		return p;
	}
	public void free() {
		m_controlLoop.cancel();
		synchronized (this) {
			m_controlLoop = null;
		}
		super.free();
	}
	public void runCalibration() {
		m_calLog.open("Calibration" + m_file, "Timestamp, Distance, Velocity" + "\n");
		synchronized(this) {
			m_pidOutput.pidWrite(1.0);
		}
		double currentCal = Timer.getFPGATimestamp();
		double currentDist = m_pidInput.pidGet();
		double currentV = (currentDist-m_lastDist)/(currentCal-m_lastCal);
		m_calLog.write(currentCal-m_calStart + ", " + currentDist + ", " + currentV + "\n");
		if(currentV <= (m_lastV*1.01)) {
			m_count +=1;
			if(m_count >= 10) {
				stopCalibration();
			}
		}else {
			m_count = 0;
		}
		m_lastDist = currentDist;
		m_lastCal = currentCal;
		m_lastV = currentV;
	}
	public void startCalibration() {
		m_calibrating = true;
		m_lastCal = m_calStart = Timer.getFPGATimestamp();
		m_lastDist = m_pidInput.pidGet();

	}
	public void stopCalibration() {
		m_calLog.close();
		synchronized(this) {
			m_pidOutput.pidWrite(0.0);
			m_calibrating = false;
		}
	}

	public void runPlan() {
		double currentTime = Timer.getFPGATimestamp();
		m_currentWaypoint = getCurrentWaypoint(currentTime - m_initTime);

		if(m_currentWaypoint == null) {
			synchronized(this) {
				m_planFinished = true;
				if(!m_dwell) {
					m_pidOutput.pidWrite(0.0);
					return;
				}
			}
		} else {
			setSetpoint(m_currentWaypoint.m_position + m_initPos);
		}
		calculate();

		synchronized(this) {
			if(isEnabled()) {
				m_pidOutput.pidWrite(m_result*m_invertMultiplier);
			}
		}		  
	}

	public boolean isPlanFinished() {
		return m_planFinished;
	}
	
	protected double calculateFeedForward() {
		if(m_motionPlanEnabled && m_currentWaypoint != null) {
    		return (m_currentWaypoint.m_expectedAcceleration * m_kA) + (m_currentWaypoint.m_expectedVelocity * m_kV);
    	}
    	
    	return 0.0;

	}

	private class PIDTask extends TimerTask {

		@Override
		public void run() {
			if(isEnabled()) {
				if(m_calibrating) {
					runCalibration();
				}else if(m_motionPlanEnabled){
					runPlan();
				}

			}
		}
	}
}

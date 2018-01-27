package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class Autonomous {
	Robot robot;
	Timer timer;
	double initTime;
	int stage = 0;
	StageDataElement[] stageData;
	
	class StageDataElement {
		AutoBlueprint blueprint;
		boolean entered;
	}
	
	public Autonomous(Robot robot) {
		this.robot = robot;
	}
	
	/** Java has a dynamic array class called ArrayList, would it be better to use this over the static-length Array? **/
	public void setBlueprints(AutoBlueprint[] b) {
		stageData = new StageDataElement[b.length];
		
		for(int i = 0; i < b.length; i++) {
			stageData[i] = new StageDataElement();
			
			stageData[i].blueprint = b[i];
			stageData[i].entered = false;
		}
	}
	
	public void start() {
		//robot.gyro.reset();
		robot.leftEncoder.reset();
		robot.rightEncoder.reset();
		
		robot.leftControl.reset();
		robot.rightControl.reset();
		
        /*if(robot.allianceChooser.getSelected().equals("-1.0")) {
        	robot.allianceMult = -1.0;
        } else {
        	robot.allianceMult = 1.0;
        }
        
        System.out.println(robot.allianceMult);*/
				
		stage = 0;
		initTime = Timer.getFPGATimestamp();
		
		timer = new Timer();
		timer.reset();
		timer.start();
	}
	
	public boolean checkStageTimeout() {
		if (stage < 0 || stage >= stageData.length) {
			robot.motor0.set(0.0);
			robot.motor1.set(0.0);
			robot.motor2.set(0.0);
			robot.motor3.set(0.0);
			robot.motor4.set(0.0);
			robot.motor5.set(0.0);
			
			return true;
		}
	
		if (timer.get() > stageData[stage].blueprint.m_timeout) {

			System.out.printf("stage %d timed out\n", stage);
			nextStage();
			return true;
		}
		return false;
	}
	
	public void nextStage() {
		System.out.printf("Stage Finished: %d\tTime: %f\tTotal Time: %f\n",stage,timer.get(),Timer.getFPGATimestamp() - initTime);
		//System.out.println("ex:" + robot.lEncoder + " ey: " + robot.encoderThread.getY() + " r: " + robot.navx.getAngle());
		timer.reset();
		stage++;
		
		if(stage >= stageData.length) {
			end();
		}
	}
	
	public void end() {
		System.out.println("-----");
		System.out.printf("Auto Finished:\tTotal Time: %f\n",Timer.getFPGATimestamp() - initTime);
		
		robot.rightControl.reset();
		robot.leftControl.reset();
	}
	
	public void init() {
		System.out.println("Override me!");
	}
	
	public void periodic() {		
		if(checkStageTimeout()) {
			return;
		}
		if(!stageData[stage].entered) {
			stageData[stage].blueprint.m_start.getAsInt();
			stageData[stage].entered = true;
		}
			
		stageData[stage].blueprint.m_periodic.getAsInt();
	}

}

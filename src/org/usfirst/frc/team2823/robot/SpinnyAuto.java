package org.usfirst.frc.team2823.robot;

public class SpinnyAuto extends Autonomous {
	
	public SpinnyAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		AutoBlueprint[] blueprints = new AutoBlueprint[] {new AutoBlueprint(7.0, this::stageStart, this::stagePeriodic)};
		setBlueprints(blueprints);
		
		start();
	}
	
	//drive across the baseline
	public int stageStart() {
		//run entry code	
		robot.leftControl.configureGoal(90, 300, 300, false);
		robot.rightControl.configureGoal(-90, 300, 300, false);
			
		robot.leftControl.enable();
		robot.rightControl.enable();
		
		return 0;
		
		//move on to the next stage once plan is complete
	}
	
	public int stagePeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
			
			nextStage();
		}
		return 0;
	}
}

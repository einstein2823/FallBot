package org.usfirst.frc.team2823.robot;

public class GoGoFallbotAuto extends Autonomous {
	
	public GoGoFallbotAuto(Robot robot) {
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
		robot.traj = new TrajectoryPlanner();
		robot.traj.generate();
		
		robot.leftControl.configureTrajectory(robot.traj.getLeftTrajectory(), false);
		robot.rightControl.configureTrajectory(robot.traj.getRightTrajectory(), false);
		
		robot.leftControl.enable();
		robot.rightControl.enable();

		return 0;
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

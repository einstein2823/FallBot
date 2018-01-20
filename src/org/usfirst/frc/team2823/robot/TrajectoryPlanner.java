package org.usfirst.frc.team2823.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectoryPlanner {
	static SnazzyLog log = new SnazzyLog();
	private static Trajectory m_left;
	private static Trajectory m_right;
	
    public static void generate() {
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 120, 275, 300);
        Waypoint[] points = new Waypoint[] {
                //new Waypoint(-4, -1, Pathfinder.d2r(-45)),
                new Waypoint(0,0,0),
                new Waypoint(48, 12, Pathfinder.d2r(30))
        };

        Trajectory trajectory = Pathfinder.generate(points, config);

        // Wheelbase Width = 0.5m
        TankModifier modifier = new TankModifier(trajectory).modify(17.5);

        // Do something with the new Trajectories...
        m_left = modifier.getLeftTrajectory();
        m_right = modifier.getRightTrajectory();
        
        for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = trajectory.get(i);
            
            log.open("Trajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
        for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = m_left.get(i);
            
            log.open("LeftTrajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
        for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = m_right.get(i);
            
            log.open("RightTrajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
    }

}
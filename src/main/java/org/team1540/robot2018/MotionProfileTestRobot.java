package org.team1540.robot2018;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import org.team1540.base.motionprofiling.RunMotionProfiles;
import org.team1540.robot2018.subsystems.DriveTrain;

public class MotionProfileTestRobot extends IterativeRobot {
  private DriveTrain driveTrain = new DriveTrain();
  RunMotionProfiles profileCommand;

  @Override
  public void autonomousInit() {
    driveTrain.zeroEncoders();
    Waypoint[] points = new Waypoint[]{
        new Waypoint(1, 0, 0.001),
        new Waypoint(20, 0, Math.PI / 4)
    };

    Trajectory.Config cfg = new Config(FitMethod.HERMITE_CUBIC, Config.SAMPLES_FAST, 0.03, 3, 2, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, cfg);

    TankModifier modifier = new TankModifier(trajectory).modify(0.61);

    profileCommand = new RunMotionProfiles(
        driveTrain.createLeftProfileProperties(modifier.getLeftTrajectory()),
        driveTrain.createRightProfileProperties(modifier.getRightTrajectory())
    );

    profileCommand.start();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left velocity", driveTrain.getLeftVelocity());
    SmartDashboard.putNumber("Left position", driveTrain.getLeftPosition());
    SmartDashboard.putNumber("Right velocity", driveTrain.getRightVelocity());
    SmartDashboard.putNumber("Right position", driveTrain.getRightPosition());
    Scheduler.getInstance().run();
  }
}

package org.team1540.robot2018;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.io.File;
import org.team1540.base.adjustables.Tunable;
import org.team1540.base.util.SimpleCommand;
import org.team1540.base.util.SimpleLoopCommand;
import org.team1540.robot2018.motion.FollowProfile;
import org.team1540.robot2018.subsystems.DriveTrain;

public class RobotRecording extends IterativeRobot {

  private final DriveTrain drivetrain = new DriveTrain();
  private RecordProfile drivetrainRecording = new RecordProfile(drivetrain.driveLeftMotorA,
      drivetrain.driveRightMotorA);
  public static String path = "/home/lvuser/generated-profiles/";

  private Trajectory leftTrajectory;
  private Trajectory rightTrajectory;

  @Tunable("Time dialation")
  public double timeDialation = 1;

  public RobotRecording() {
    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotInit() {
    SmartDashboard.putData(new SimpleCommand("Save Recorded Profile", this::saveTrajectories));
  }

  @Override
  public void teleopInit() {
    new SimpleLoopCommand("Anit-PID", () -> {}, drivetrain);
    drivetrain.setBrake(false);
    drivetrainRecording = new RecordProfile(drivetrain.driveLeftMotorA,
        drivetrain.driveRightMotorA);
    drivetrainRecording.start();
  }

  @Override
  public void disabledInit() {
    try {
      drivetrainRecording.stopRunning();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    leftTrajectory = new Trajectory(drivetrainRecording.getMotors().get(drivetrain.driveLeftMotorA).segments.toArray(new Segment[0]));
    rightTrajectory = new Trajectory(drivetrainRecording.getMotors().get(drivetrain.driveRightMotorA).segments.toArray(new Segment[0]));

    RecordProfile.timeDialateTrajectory(leftTrajectory, timeDialation);
    RecordProfile.timeDialateTrajectory(rightTrajectory, timeDialation);

    // String folderPath = path + System.currentTimeMillis() + "/";
    // for (Entry<ChickenTalon, Storage> entry : drivetrainRecording.getMotors().entrySet()) {
    //   processSegments(entry.getValue().segments.toArray(new Segment[entry.getValue().segments
    //       .size()]), folderPath + entry.getKey().getDeviceID());
    // }
  }

  public void saveTrajectories() {
    File leftFile = new File(path + System.currentTimeMillis() + "/left.csv");
    File rightFile = new File(path + System.currentTimeMillis() + "/right.csv");
    //noinspection ResultOfMethodCallIgnored
    leftFile.getParentFile().mkdirs();
    Pathfinder.writeToCSV(leftFile, leftTrajectory);
    Pathfinder.writeToCSV(rightFile, rightTrajectory);
  }

  @Override
  public void autonomousInit() {
    if (leftTrajectory != null && rightTrajectory != null) {
      new FollowProfile(leftTrajectory, rightTrajectory);
    }
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}

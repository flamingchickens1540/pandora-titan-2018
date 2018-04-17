package org.team1540.robot2018;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.io.File;
import java.util.Map.Entry;
import org.team1540.base.util.SimpleLoopCommand;
import org.team1540.base.wrappers.ChickenTalon;
import org.team1540.robot2018.RecordProfile.Storage;
import org.team1540.robot2018.subsystems.DriveTrain;

public class RobotRecording extends IterativeRobot {

  private final DriveTrain drivetrain = new DriveTrain();
  private RecordProfile drivetrainRecording = new RecordProfile(drivetrain.driveLeftMotorA,
      drivetrain.driveRightMotorA);
  public static String path = "/home/lvuser/generated-profiles/";

  public RobotRecording() {
    LiveWindow.disableAllTelemetry();
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
    String folderPath = path + System.currentTimeMillis() + "/";
    for (Entry<ChickenTalon, Storage> entry : drivetrainRecording.getMotors().entrySet()) {
      processSegments(entry.getValue().segments.toArray(new Segment[entry.getValue().segments
          .size()]), folderPath + entry.getKey().getDeviceID());
    }
  }

  public void processSegments(Segment[] segments, String path){
    if (segments.length > 1) {
      // Writing out
      Trajectory thisTrajectory = new Trajectory(segments);
      // RecordProfile.timeDialateTrajectory(thisTrajectory, timeDialation);
      File file = new File(path + ".csv");
      //noinspection ResultOfMethodCallIgnored
      file.getParentFile().mkdirs();
      Pathfinder.writeToCSV(file, thisTrajectory);
    }
  }
}

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
  private final RecordProfile drivetrainRecording = new RecordProfile(drivetrain.driveLeftMotorA,
      drivetrain.driveRightMotorA);
  public static String path = "/home/lvuser/profiles/";

  public RobotRecording() {
    LiveWindow.disableAllTelemetry();
    drivetrainRecording.start();
  }

  @Override
  public void teleopInit() {
    drivetrainRecording.reset();
    drivetrainRecording.running = true;
    new SimpleLoopCommand("Anit-PID", () -> {}, drivetrain);
    drivetrain.disableBrake();
  }

  @Override
  public void disabledInit() {
    drivetrainRecording.running = false;
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    drivetrainRecording.interrupt();
    for (Entry<ChickenTalon, Storage> entry : drivetrainRecording.getMotors().entrySet()) {
      if (entry.getValue().segments.size() > 0) {
        Trajectory thisTrajectory = new Trajectory(entry.getValue().segments.toArray(new
            Segment[entry.getValue().segments.size()]));
        // RecordProfile.timeDialateTrajectory(thisTrajectory, timeDialation);
        Pathfinder.writeToCSV(new File(path + System.currentTimeMillis() + "/" + entry.getKey()
            .getDeviceID() + ".csv"), thisTrajectory);
      }
    }
  }

}

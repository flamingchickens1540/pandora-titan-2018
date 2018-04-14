package org.team1540.robot2018;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.team1540.base.util.SimpleLoopCommand;
import org.team1540.base.wrappers.ChickenTalon;
import org.team1540.robot2018.RecordProfile.Storage;
import org.team1540.robot2018.subsystems.DriveTrain;

public class RobotRecording extends IterativeRobot {

  private final DriveTrain drivetrain = new DriveTrain();
  private RecordProfile drivetrainRecording = new RecordProfile(drivetrain.driveLeftMotorA,
      drivetrain.driveRightMotorA);
  public static String path = "/home/lvuser/generated-profiles/";

  public int degreesToFit = 4;
  public int pointsToUse = 2;

  public RobotRecording() {
    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void teleopInit() {
    new SimpleLoopCommand("Anit-PID", () -> {}, drivetrain);
    drivetrain.disableBrake();
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
      if (entry.getValue().segments.size() > 1) {
        List<Double> times = new ArrayList<>(2 * pointsToUse + 1);
        List<Double> velocities = new ArrayList<>(2 * pointsToUse + 1);

        for (int i = 0; i < entry.getValue().segments.size(); i++) {
          // Acceleration and jerk interpolation
          // TODO use Apache math fun? Way more flexibility, potentially better fits

          times.clear();
          velocities.clear();
          double timeTotal = 0;
          double targetTime = 0;
          for (int j = -pointsToUse; j <= pointsToUse; j++) {
            int newIndex = i+j;
            if (!(newIndex < 0 || newIndex > entry.getValue().segments.size() - 1)) {
              Segment segment = entry.getValue().segments.get(newIndex);
              timeTotal += segment.dt;
              if (j == 0) {
                targetTime = timeTotal;
              }
              times.add(timeTotal);
              velocities.add(segment.velocity);
            }
          }

          // Set the acceleration to the value of the derivative of the interpolated function
          entry.getValue().segments.get(i).acceleration =
              new SplineInterpolator().interpolate(times.stream().mapToDouble
              (Double::doubleValue).toArray(), velocities.stream().mapToDouble
              (Double::doubleValue).toArray()).derivative().value(targetTime);

        }

        // Writing out
        Trajectory thisTrajectory = new Trajectory(entry.getValue().segments.toArray(new
            Segment[entry.getValue().segments.size()]));
        // RecordProfile.timeDialateTrajectory(thisTrajectory, timeDialation);
        File file = new File( folderPath + entry.getKey().getDeviceID() + ".csv");
        //noinspection ResultOfMethodCallIgnored
        file.getParentFile().mkdirs();
        Pathfinder.writeToCSV(file, thisTrajectory);
      }
    }
  }
}

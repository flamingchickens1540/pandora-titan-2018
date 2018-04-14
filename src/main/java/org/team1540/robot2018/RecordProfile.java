package org.team1540.robot2018;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.team1540.base.wrappers.ChickenTalon;

public class RecordProfile extends Thread {

  private Map<ChickenTalon, Storage> motors= new HashMap<>();
  private Timer timer = new Timer();
  private boolean running = true;
  public long dt = 10;
  public int degreesToFit = 4;
  public int pointsToUse = 2;

  public RecordProfile(ChickenTalon... motors) {
    for (ChickenTalon motor : motors) {
      this.motors.put(motor, new Storage());
    }
    timer.reset();
  }

  @Override
  public void run() {
    timer.start();
    for (Storage storage : motors.values()) {
      storage.lastTime = timer.get();
    }
    while (running) {
      for (Entry<ChickenTalon, Storage> entry : motors.entrySet()) {
        double time = timer.get();
        entry.getValue().segments.add(new Segment(
            time - entry.getValue().lastTime,
            0,
            0,
            entry.getKey().getSelectedSensorPosition(),
            entry.getKey().getSelectedSensorVelocity(),
            0,
            0,
            Robot.navx.getYaw()
        ));
        entry.getValue().lastTime = time;
      }
      try {
        Thread.sleep(dt);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  public class Storage {
    public List<Segment> segments = new LinkedList<>();
    public double lastTime = 0;
  }

  public void addAcceleration(List<Segment> segments) {
    List<Double> times = new ArrayList<>(2 * pointsToUse + 1);
    List<Double> velocities = new ArrayList<>(2 * pointsToUse + 1);

    for (int i = 0; i < segments.size(); i++) {
      // Acceleration and jerk interpolation

      times.clear();
      velocities.clear();
      double timeTotal = 0;
      double targetTime = 0;
      for (int j = -pointsToUse; j <= pointsToUse; j++) {
        int newIndex = i + j;
        if (!(newIndex < 0 || newIndex > segments.size() - 1)) {
          Segment segment = segments.get(newIndex);
          timeTotal += segment.dt;
          if (j == 0) {
            targetTime = timeTotal;
          }
          times.add(timeTotal);
          velocities.add(segment.velocity);
        }
      }

      // Set the acceleration to the value of the derivative of the interpolated function
      segments.get(i).acceleration =
          new SplineInterpolator().interpolate(times.stream().mapToDouble
              (Double::doubleValue).toArray(), velocities.stream().mapToDouble
              (Double::doubleValue).toArray()).derivative().value(targetTime);
    }
  }

  public static void timeDialateTrajectory(Trajectory originalTrajectory, double dialation) {
    for (Segment segment : originalTrajectory.segments) {
      segment.dt /= dialation;
      segment.position *= dialation;
      segment.velocity *= Math.pow(dialation, 2);
      segment.acceleration *= Math.pow(dialation, 3);
      segment.jerk *= Math.pow(dialation, 4);
    }
  }

  public Map<ChickenTalon, Storage> getMotors() {
    return motors;
  }

  public void stopRunning() throws InterruptedException {
    this.running = false;
    Thread.sleep(1000);
    for (Storage storage : motors.values()) {
      addAcceleration(storage.segments);
    }
  }

}

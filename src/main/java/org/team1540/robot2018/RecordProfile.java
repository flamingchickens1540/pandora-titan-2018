package org.team1540.robot2018;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import org.team1540.base.wrappers.ChickenTalon;

public class RecordProfile extends Thread {

  private Map<ChickenTalon, Storage> motors= new HashMap<>();
  private Timer timer = new Timer();
  public boolean running = true;

  public RecordProfile(ChickenTalon... motors) {
    for (ChickenTalon motor : motors) {
      this.motors.put(motor, new Storage());
    }
  }

  public void reset() {
    timer.reset();
    timer.start();
    for (Storage storage : motors.values()) {
      storage.lastTime = timer.get();
    }
    super.start();
  }

  @Override
  public void start() {
    reset();
  }

  @Override
  public void run() {
    while (true) {
      if (running) {
        for (Entry<ChickenTalon, Storage> entry : motors.entrySet()) {
          entry.getValue().segments.add(new Segment(
              timer.get() - entry.getValue().lastTime,
              -1,
              -1,
              entry.getKey().getSelectedSensorPosition(),
              entry.getKey().getSelectedSensorVelocity(),
              -1,
              -1,
              Robot.navx.getYaw()
          ));
        }
      }
    }
  }

  public class Storage {
    public List<Segment> segments = new LinkedList<>();
    public double lastTime = 0;
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

}

package org.team1540.robot2018.motion;

import static java.lang.Double.max;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.StrictMath.min;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;

public class FollowProfile extends Command {
  private Notifier loop = new Notifier(this::run);
  private Timer timer = new Timer();
  private Trajectory left;
  private Trajectory right;
  private boolean finished;

  public FollowProfile(Trajectory left, Trajectory right) {
    this.left = left;
    this.right = right;

    requires(Robot.drivetrain);
  }

  private void run() {
    //TODO: perform linear interpolation between trajectory points?
    Segment leftSegment = getCurrentSegment(left, timer.get());
    Segment rightSegment = getCurrentSegment(right, timer.get());

    double leftVelocity = (leftSegment.velocity * Tuning.drivetrainEncoderTPU) * 0.1;

    double robotHeading = 2 * PI - Math.toRadians(
        Robot.navx.getYaw() < 0 ? 360 + Robot.navx.getYaw() : Robot.navx.getYaw());
    double desiredHeading = leftSegment.heading;

    double regularError = robotHeading - desiredHeading;
    double differentError =
        (2 * PI - max(robotHeading, desiredHeading)) - min(robotHeading, desiredHeading);

    double headingError = abs(regularError) < abs(differentError) ? regularError : differentError;

    // heading is negated for the left side only so that negative heading errors (i.e. too far right)
    // result in the left side slowing but the right side speeding up
    double leftVelocitySetpoint = leftVelocity + getVelocityModification(
        (leftSegment.acceleration * Tuning.drivetrainEncoderTPU) * 0.1,
        leftVelocity - Robot.drivetrain.getLeftVelocity(),
        (leftSegment.position * Tuning.drivetrainEncoderTPU) - Robot.drivetrain.getLeftPosition(),
        headingError);

    double rightVelocity = (rightSegment.velocity * Tuning.drivetrainEncoderTPU) * 0.1;

    double rightVelocitySetpoint = rightVelocity + getVelocityModification(
        (rightSegment.acceleration * Tuning.drivetrainEncoderTPU) * 0.1,
        rightVelocity - Robot.drivetrain.getLeftVelocity(),
        (rightSegment.position * Tuning.drivetrainEncoderTPU) - Robot.drivetrain.getRightPosition(),
        -headingError);

    Robot.drivetrain.setLeftVelocity(leftVelocitySetpoint);
    Robot.drivetrain.setRightVelocity(rightVelocitySetpoint);
    System.out.println("Gyro Angle " + robotHeading);
    System.out.println("Intended Heading " + desiredHeading);

    System.out.println("Error" + headingError);
    // System.out.println("Heading Correction "+ (Tuning.profileHeadingP *(leftSegment.heading - Math.toRadians(-Robot.navx.getYaw()))));
  }

  @Override
  protected boolean isFinished() {
    return finished;
  }

  // stolen from Jonathan's follower
  private Segment getCurrentSegment(Trajectory trajectory, double currentTime) {
    // Start from the current time and find the closest point.
    int startIndex = Math.toIntExact(Math.round(currentTime / trajectory.segments[0].dt));

    int length = trajectory.segments.length;
    int index = startIndex;
    if (startIndex >= length - 1) {
      index = length - 1;
      finished = true;
    }
    return trajectory.segments[index];
  }

  private Segment getInterpolatedSegment(Trajectory trajectory, double currentTime) {
    // Start from the current time and find the closest point.
    int startIndex = (int) Math.floor(currentTime / trajectory.segments[0].dt);
    int endIndex = (int) Math.ceil(currentTime / trajectory.segments[0].dt);

    int length = trajectory.segments.length;
    if (endIndex >= length - 1) {
      startIndex = length - 1;
      endIndex = length - 1;
      finished = true;
    }

    double timeSinceLower = currentTime % trajectory.segments[0].dt;
    Segment lower = trajectory.segments[startIndex];
    Segment upper = trajectory.segments[endIndex];

    //noinspection SuspiciousNameCombination
    return new Segment(
        0, // unused by users of this method
        interpolate(timeSinceLower, 0, lower.x, upper.dt, upper.x),
        interpolate(timeSinceLower, 0, lower.y, upper.dt, upper.y),
        interpolate(timeSinceLower, 0, lower.position, upper.dt, upper.position),
        interpolate(timeSinceLower, 0, lower.velocity, upper.dt, upper.velocity),
        interpolate(timeSinceLower, 0, lower.acceleration, upper.dt, upper.acceleration),
        interpolate(timeSinceLower, 0, lower.jerk, upper.dt, upper.jerk),
        interpolate(timeSinceLower, 0, lower.heading, upper.dt, upper.heading));
  }

  /**
   * Interpolates between two points using linear interpolation.
   */
  private double interpolate(double x, double lowX, double lowY, double highX,
      double highY) {
    return (x - lowX) * (highY - lowY) / (highX - lowX) + lowY;
  }

  private static double getVelocityModification(double trajAccelSetpoint, double velocityError, double positionError, double headingError) {
    return Tuning.profileHeadingP * headingError +
        Tuning.profileAccelP * trajAccelSetpoint +
        Tuning.profileVelocityP * velocityError +
        Tuning.profilePositionP * positionError;
  }

  @Override
  protected void initialize() {
    timer.reset();
    Robot.navx.zeroYaw();
    timer.start();
    loop.startPeriodic(Tuning.profileLoopFrequency);
    finished = false;
  }

  @Override
  protected void end() {
    loop.stop();
    System.out.println("Profile Done!");
  }
}
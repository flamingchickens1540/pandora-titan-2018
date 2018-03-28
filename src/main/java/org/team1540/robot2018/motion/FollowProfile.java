package org.team1540.robot2018.motion;

import static java.lang.Double.max;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.StrictMath.min;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import org.team1540.base.Utilities;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;

public class FollowProfile extends Command {
  private Notifier loop = new Notifier(this::run);
  private EncoderFollower left;
  private EncoderFollower right;
  private boolean finished;

  public FollowProfile(Trajectory left, Trajectory right) {
    this.left = new EncoderFollower(left);
    this.right = new EncoderFollower(right);

    requires(Robot.drivetrain);
  }

  private void run() {
    double leftVelocity =
        Utilities.constrain(left.calculate((int) Robot.drivetrain.getLeftPosition()), 1);
    double rightVelocity =
        Utilities.constrain(right.calculate((int) Robot.drivetrain.getRightPosition()), 1);

    double robotHeading = 2 * PI - Math.toRadians(
        Robot.navx.getYaw() < 0 ? 360 + Robot.navx.getYaw() : Robot.navx.getYaw());
    double desiredHeading = left.getHeading();

    double regularError = robotHeading - desiredHeading;
    double differentError =
        (2 * PI - max(robotHeading, desiredHeading)) - min(robotHeading, desiredHeading);

    double headingError = abs(regularError) < abs(differentError) ? regularError : differentError;

    // heading is negated for the left side only so that negative heading errors (i.e. too far right)
    // result in the left side slowing but the right side speeding up
    double leftVelocitySetpoint = leftVelocity - (Tuning.profileHeadingP * headingError);

    double rightVelocitySetpoint = rightVelocity + (Tuning.profileHeadingP * headingError);

    Robot.drivetrain.setLeftPercent(leftVelocitySetpoint);
    Robot.drivetrain.setRightPercent(rightVelocitySetpoint);
    System.out.println("Gyro Angle " + robotHeading);
    System.out.println("Intended Heading " + desiredHeading);

    System.out.println("Error" + headingError);
    // System.out.println("Heading Correction "+ (Tuning.profileHeadingP *(leftSegment.heading - Math.toRadians(-Robot.navx.getYaw()))));
  }

  @Override
  protected boolean isFinished() {
    return left.isFinished() && right.isFinished();
  }

  @Override
  protected void initialize() {
    Robot.navx.zeroYaw();

    // this is hacky but I've looked through the source code and this has the same result as setting a pure TPU value
    left.configureEncoder((int) Robot.drivetrain.getLeftPosition(), 1,
        Tuning.drivetrainEncoderTPU / Math.PI);
    right.configureEncoder((int) Robot.drivetrain.getRightPosition(), 1,
        Tuning.drivetrainEncoderTPU / Math.PI);

    left.configurePIDVA(Tuning.profileP, 0, Tuning.profileD, (1.0
        / Tuning.drivetrainMaxVelocity), Tuning.profileAccelP);
    right.configurePIDVA(Tuning.profileP, 0, Tuning.profileD, (1.0
        / Tuning.drivetrainMaxVelocity), Tuning.profileAccelP);

    loop.startPeriodic(Tuning.profileLoopFrequency);
  }

  @Override
  protected void end() {
    loop.stop();
    System.out.println("Profile Done!");
  }
}

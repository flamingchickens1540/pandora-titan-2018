package org.team1540.robot2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.LinkedList;
import java.util.List;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.adjustables.Tunable;
import org.team1540.base.wrappers.ChickenTalon;

public class AccelerationCharacterizationRobot extends IterativeRobot {
  private ChickenTalon driveLeftMotorA = new ChickenTalon(RobotMap.DRIVE_LEFT_A);
  private ChickenTalon driveLeftMotorB = new ChickenTalon(RobotMap.DRIVE_LEFT_B);
  private ChickenTalon driveLeftMotorC = new ChickenTalon(RobotMap.DRIVE_LEFT_C);
  private ChickenTalon[] driveLeftMotors = new ChickenTalon[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC};
  private ChickenTalon driveRightMotorA = new ChickenTalon(RobotMap.DRIVE_RIGHT_A);
  private ChickenTalon driveRightMotorB = new ChickenTalon(RobotMap.DRIVE_RIGHT_B);
  private ChickenTalon driveRightMotorC = new ChickenTalon(RobotMap.DRIVE_RIGHT_C);
  private ChickenTalon[] driveRightMotors = new ChickenTalon[]{driveRightMotorA, driveRightMotorB, driveRightMotorC};
  private ChickenTalon[] driveMotorAll = new ChickenTalon[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC, driveRightMotorA, driveRightMotorB, driveRightMotorC};
  private ChickenTalon[] driveMotorMasters = new ChickenTalon[]{driveLeftMotorA, driveRightMotorA};

  private PrintWriter csvWriter = null;

  private Notifier notifier = new Notifier(this::run);

  private void run() {
    if (!isOperatorControl() && csvWriter != null) {
      csvWriter.close();
      csvWriter = null;
    }
    Scheduler.getInstance().run();
    if (leftVelocities.size() == 4) {
      leftVelocities.remove(0);
      rightVelocities.remove(0);
      leftVoltages.remove(0);
      rightVoltages.remove(0);
      times.remove(0);
    }
    double leftVelocity = driveLeftMotorA.getSelectedSensorVelocity();
    double rightVelocity = driveRightMotorA.getSelectedSensorVelocity();

    leftVelocities.add(leftVelocity);
    rightVelocities.add(rightVelocity);

    double accelCausingVoltageLeft =
        driveLeftMotorA.getMotorOutputVoltage() - (Tuning.profileVelocityF * leftVelocity
            + Tuning.profileVelocityIntercept);
    double accelCausingVoltageRight =
        driveRightMotorA.getMotorOutputVoltage() - (Tuning.profileVelocityF * rightVelocity
            + Tuning.profileVelocityIntercept);
    leftVoltages.add(accelCausingVoltageLeft);
    rightVoltages.add(accelCausingVoltageRight);
    times.add((double) System.currentTimeMillis() / 1000.0);

    if (leftVelocities.size() == 4) {
      double lAccel = bestFitSlope(times, leftVelocities);
      double rAccel = bestFitSlope(times, rightVelocities);
      SmartDashboard.putNumber("Left Accel", lAccel);
      SmartDashboard.putNumber("Right Accel", rAccel);
    }

    if (joystick.getRawButton(1)) { // if button A is pressed
      if (csvWriter == null) {
        // create a new CSV writer, reset everything
        try {
          csvWriter = new PrintWriter(new File(
              "/home/lvuser/dtmeasure/measureaccel-" + System.currentTimeMillis() + ".csv"));
          csvWriter.println("lvoltage,laccel,rvoltage,raccel");
        } catch (FileNotFoundException e) {
          throw new RuntimeException(e);
        }
        driveLeftMotorA.set(ControlMode.PercentOutput, 0);
        driveRightMotorA.set(ControlMode.PercentOutput, 0);
      } else {
        SmartDashboard.putNumber("Left Output", driveLeftMotorA.getMotorOutputPercent());
        SmartDashboard.putNumber("Left Velocity", leftVelocity);
        SmartDashboard.putNumber("Right Output", driveRightMotorA.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Velocity", rightVelocity);

        if (leftVelocities.size() == 4) {
          double lAccel = bestFitSlope(times, leftVelocities);
          double rAccel = bestFitSlope(times, rightVelocities);
          csvWriter.println(
              leftVoltages.get(1) + "," + lAccel + "," + rightVoltages.get(1) + "," + rAccel);
          System.out.println(leftVelocities.toString());
          System.out.println(times.toString());
          System.out.println(lAccel);
        }
        driveLeftMotorA.set(ControlMode.PercentOutput, setpoint);
        driveRightMotorA.set(ControlMode.PercentOutput, setpoint);
      }
    } else {
      if (csvWriter != null) {
        csvWriter.close();
        csvWriter = null;
      }
      driveLeftMotorA.set(ControlMode.PercentOutput, 0);
      driveRightMotorA.set(ControlMode.PercentOutput, 0);
    }
  }

  private Joystick joystick = new Joystick(0);

  private List<Double> leftVelocities = new LinkedList<>();
  private List<Double> leftVoltages = new LinkedList<>();
  private List<Double> rightVelocities = new LinkedList<>();
  private List<Double> rightVoltages = new LinkedList<>();
  private List<Double> times = new LinkedList<>();

  private Double[] zeroLenArr = new Double[0];

  @Tunable("Setpoint")
  public double setpoint = 0.6;

  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(this);
    reset();
    notifier.startPeriodic(0.01);
  }

  @Override
  public void robotPeriodic() {

  }

  @SuppressWarnings("SuspiciousNameCombination")
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void disabledInit() {
  }

  @SuppressWarnings("Duplicates")
  public void reset() {
    driveLeftMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    driveRightMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    driveLeftMotorA.setSensorPhase(Tuning.isPandora);

    for (ChickenTalon talon : driveLeftMotors) {
      talon.setInverted(false);
    }

    driveRightMotorA.setSensorPhase(true);

    for (ChickenTalon talon : driveRightMotors) {
      talon.setInverted(true);
    }

    driveLeftMotorB.set(ControlMode.Follower, driveLeftMotorA.getDeviceID());
    driveLeftMotorC.set(ControlMode.Follower, driveLeftMotorA.getDeviceID());

    driveRightMotorB.set(ControlMode.Follower, driveRightMotorA.getDeviceID());
    driveRightMotorC.set(ControlMode.Follower, driveRightMotorA.getDeviceID());

    for (ChickenTalon talon : driveMotorAll) {
      talon.setBrake(true);
    }

    for (ChickenTalon talon : driveMotorAll) {
      talon.configClosedloopRamp(0);
      talon.configOpenloopRamp(0);
      talon.configPeakOutputForward(1);
      talon.configPeakOutputReverse(-1);
      talon.enableCurrentLimit(false);
    }
  }

  private static double bestFitSlope(List<Double> xVals, List<Double> yVals) {
    double avgX = xVals.stream().mapToDouble(x -> x).sum() / xVals.size();
    double avgY = yVals.stream().mapToDouble(y -> y).sum() / yVals.size();

    double sumXY = 0;
    double sumXSquared = 0;
    for (int i = 0; i < xVals.size(); i++) {
      sumXY += (xVals.get(i) - avgX) * (yVals.get(i) - avgY);
      sumXSquared += (xVals.get(i) - avgX) * (xVals.get(i) - avgX);
    }

    return sumXY / sumXSquared;
  }
}

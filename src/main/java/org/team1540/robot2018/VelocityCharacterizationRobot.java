package org.team1540.robot2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.adjustables.Telemetry;
import org.team1540.base.adjustables.Tunable;
import org.team1540.base.wrappers.ChickenTalon;

public class VelocityCharacterizationRobot extends IterativeRobot {
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

  private Joystick joystick = new Joystick(0);

  @Telemetry("Current Applied Output")
  public double appliedOutput;
  @Tunable("Voltage Ramp Rate")
  public double rampRate = 0.020833333;

  public long lastTime;

  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(this);
    reset();
  }

  @Override
  public void robotPeriodic() {
    if (!isOperatorControl() && csvWriter != null) {
      csvWriter.close();
      csvWriter = null;
    }
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    if (joystick.getRawButton(1)) { // if button A is pressed
      if (csvWriter == null) {
        // create a new CSV writer, reset everything
        try {
          csvWriter = new PrintWriter(new File(
              "/home/lvuser/dtmeasure/measure-" + System.currentTimeMillis() + ".csv"));
          csvWriter.println("lvoltage,lvelocity,rvoltage,rvelocity");
        } catch (FileNotFoundException e) {
          throw new RuntimeException(e);
        }
        appliedOutput = 0;
        driveLeftMotorA.set(ControlMode.PercentOutput, 0);
        driveRightMotorA.set(ControlMode.PercentOutput, 0);
      } else {

        csvWriter.println(driveLeftMotorA.getMotorOutputVoltage() + ","
            + driveLeftMotorA.getSelectedSensorVelocity() + ","
            + driveRightMotorA.getMotorOutputVoltage() + ","
            + driveRightMotorA.getSelectedSensorVelocity());

        appliedOutput += rampRate * ((System.currentTimeMillis() - lastTime) / 1000.0);
        lastTime = System.currentTimeMillis();
        driveLeftMotorA.set(ControlMode.PercentOutput, appliedOutput);
        driveRightMotorA.set(ControlMode.PercentOutput, appliedOutput);
      }
    } else {
      appliedOutput = 0;
      if (csvWriter != null) {
        csvWriter.close();
        csvWriter = null;
      }
      driveLeftMotorA.set(ControlMode.PercentOutput, 0);
      driveRightMotorA.set(ControlMode.PercentOutput, 0);
    }
    lastTime = System.currentTimeMillis();
  }

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
}

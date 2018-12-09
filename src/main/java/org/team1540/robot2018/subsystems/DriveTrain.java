package org.team1540.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2018.OI;
import org.team1540.robot2018.RobotMap;
import org.team1540.robot2018.Tuning;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.drive.pipeline.AdvancedArcadeJoystickInput;
import org.team1540.rooster.drive.pipeline.CTREOutput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.util.SimpleLoopCommand;
import org.team1540.rooster.wrappers.ChickenTalon;

public class DriveTrain extends Subsystem {

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

  public DriveTrain() {
    reset();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new SimpleLoopCommand("Drive", new AdvancedArcadeJoystickInput(
        Tuning.drivetrainMaxVelocity,
        1,
        true,
        () -> Utilities.scale(Utilities.processDeadzone(-OI.driver.getRawAxis(OI.LEFT_Y), Tuning.axisDeadzone), Tuning.drivetrainJoystickPower),
        () -> Utilities.scale(Utilities.processDeadzone(OI.driver.getRawAxis(OI.RIGHT_X), Tuning.axisDeadzone), Tuning.drivetrainJoystickPower),
        () ->
            Utilities.scale(Utilities.processDeadzone(OI.driver.getRawAxis(OI.LEFT_TRIG), Tuning.axisDeadzone), Tuning.drivetrainJoystickPower)
                - Utilities.scale(Utilities.processDeadzone(OI.driver.getRawAxis(OI.RIGHT_TRIG), Tuning.axisDeadzone), Tuning.drivetrainJoystickPower))
        .then(new FeedForwardProcessor(1 / Tuning.drivetrainMaxVelocity, 0, 0))
        .then(new CTREOutput(driveLeftMotorA, driveRightMotorA, false)), this));
  }

  public void setLeft(ControlMode mode, double value) {
    this.driveLeftMotorA.set(mode, value);
  }

  public void setLeft(ControlMode mode, double value, double bump) {
    this.driveLeftMotorA.set(mode, value, DemandType.ArbitraryFeedForward, bump);
  }

  public void setRight(ControlMode mode, double value) {
    this.driveRightMotorA.set(mode, value);
  }

  public void setRight(ControlMode mode, double value, double bump) {
    this.driveRightMotorA.set(mode, value, DemandType.ArbitraryFeedForward, bump);
  }

  public void setLeftPercent(double value) {
    this.driveLeftMotorA.set(ControlMode.PercentOutput, value);
  }

  public void setRightPercent(double value) {
    this.driveRightMotorA.set(ControlMode.PercentOutput, value);
  }

  public void setLeftVelocity(double velocity) {
    driveLeftMotorA.set(ControlMode.Velocity, velocity);
  }

  public void setRightVelocity(double velocity) {
    driveRightMotorA.set(ControlMode.Velocity, velocity);
  }

  public void configTalonsForPosition() {
    for (ChickenTalon talon : driveMotorMasters) {
      talon.config_kP(0, Tuning.drivetrainPositionP);
      talon.config_kI(0, 0);
      talon.config_kD(0, Tuning.drivetrainPositionD);
      talon.config_kF(0, 0);
      talon.config_IntegralZone(0, 0);
    }
  }

  public void configTalonsForVelocity() {
    for (ChickenTalon talon : driveMotorMasters) {
      talon.config_kP(0, Tuning.drivetrainVelocityP);
      talon.config_kI(0, Tuning.drivetrainVelocityI);
      talon.config_kD(0, Tuning.drivetrainVelocityD);
      talon.config_kF(0, Tuning.drivetrainVelocityF);
      talon.config_IntegralZone(0, Tuning.drivetrainVelocityIZone);
    }
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

    setBrake(true);

    configTalonsForVelocity();

    for (ChickenTalon talon : driveMotorAll) {
      talon.configClosedloopRamp(Tuning.drivetrainRampRate);
      talon.configOpenloopRamp(Tuning.drivetrainRampRate);
      talon.configPeakOutputForward(1);
      talon.configPeakOutputReverse(-1);
      talon.enableCurrentLimit(false);
    }
  }

  public void enableCurrentLimiting() {
    System.out.println("Current limiting enabled!");
    for (ChickenTalon talon : driveMotorAll) {
      talon.configPeakCurrentLimit(0, 20); // Set peak to zero to just use continuous current limit
      talon.configPeakCurrentDuration(0, 20);
      talon.configContinuousCurrentLimit(Tuning.drivetrainCurrentLimit, 20);
      talon.enableCurrentLimit(true);
    }
  }

  public void setBrake(Boolean state) {
    for (ChickenTalon talon : driveMotorAll) {
      talon.setBrake(state);
    }
  }

  public double getLeftPosition() {
    return driveLeftMotorA.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return driveRightMotorA.getSelectedSensorPosition();
  }

  public double getLeftVelocity() {
    return driveLeftMotorA.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return driveRightMotorA.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
  }

  public void zeroEncoders() {
    driveLeftMotorA.setSelectedSensorPosition(0);
    driveRightMotorA.setSelectedSensorPosition(0);
  }
}

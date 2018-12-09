package org.team1540.robot2018.testing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2018.RobotMap;
import org.team1540.robot2018.Tuning;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.wrappers.ChickenTalon;

/**
 * Robot for testing the wheelbase width
 */
public class SpinnyTestRobot extends IterativeRobot {
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

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
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

    for (ChickenTalon talon : driveMotorMasters) {
      talon.config_kP(0, Tuning.drivetrainVelocityP);
      talon.config_kI(0, Tuning.drivetrainVelocityI);
      talon.config_kD(0, Tuning.drivetrainVelocityD);
      talon.config_kF(0, Tuning.drivetrainVelocityF);
      talon.config_IntegralZone(0, Tuning.drivetrainVelocityIZone);
    }

    for (ChickenTalon talon : driveMotorAll) {
      talon.configClosedloopRamp(Tuning.drivetrainRampRate);
      talon.configOpenloopRamp(Tuning.drivetrainRampRate);
    }

    // smartdash

    Command cmd = new SimpleCommand("Zero DT", () -> {
      driveLeftMotorA.setSelectedSensorPosition(0);
      driveRightMotorA.setSelectedSensorPosition(0);
    });
    cmd.setRunWhenDisabled(true);
    SmartDashboard.putData(cmd);
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    double leftDistance = driveLeftMotorA.getSelectedSensorPosition();
    double rightDistance = -driveRightMotorA.getSelectedSensorPosition();

    SmartDashboard.putNumber("Left Distance", leftDistance);
    SmartDashboard.putNumber("Right Distance", rightDistance);

    SmartDashboard.putNumber("Calculated width (assuming 10 rots)",
        (((leftDistance + rightDistance) / 2) / (10 * Math.PI)) / Tuning.drivetrainEncoderTPU);
  }

  @Override
  public void teleopPeriodic() {
    if (joystick.getRawButton(1)) {
      driveLeftMotorA.set(0.5);
      driveRightMotorA.set(-0.5);
    } else {
      driveLeftMotorA.set(ControlMode.PercentOutput, 0);
      driveRightMotorA.set(ControlMode.PercentOutput, 0);
    }
  }
}

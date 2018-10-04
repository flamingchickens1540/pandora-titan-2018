package org.team1540.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.base.wrappers.ChickenTalon;
import org.team1540.robot2018.RobotMap;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.wrist.HoldWrist;

public class Wrist extends Subsystem {

  private ChickenTalon wristMotor = new ChickenTalon(RobotMap.WRIST);

  public Wrist() {
    wristMotor.setInverted(false);

    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wristMotor.config_kP(0, Tuning.wristP);
    wristMotor.config_kI(0, Tuning.wristI);
    wristMotor.config_kD(0, Tuning.wristD);

    wristMotor.config_kF(0, Tuning.wristF);

    wristMotor.config_IntegralZone(0, Tuning.wristIzone);

    wristMotor.configMotionAcceleration(Tuning.wristMaxAccel);
    wristMotor.configMotionCruiseVelocity(Tuning.wristCruiseVelocity);
    wristMotor.configPeakOutputForward(1);
    wristMotor.configPeakOutputReverse(-1);
  }

  public double getTrajectoryPosition() {
    return wristMotor.getActiveTrajectoryPosition();
  }

  public double getError() {
    return wristMotor.getClosedLoopError();
  }

  public double getCurrent() {
    return wristMotor.getOutputCurrent();
  }

  public void setBrake(boolean state) {
    wristMotor.setBrake(state);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HoldWrist());
  }

  public void set(double value) {
    wristMotor.set(ControlMode.PercentOutput, value);
  }

  public void stop() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setSensorPosition(int position) {
    wristMotor.setSelectedSensorPosition(position);
  }

  public void resetEncoder() {
    wristMotor.setSelectedSensorPosition(0);
  }

  public void setMotionMagicPosition(double position) {
    wristMotor.set(ControlMode.MotionMagic, position);
  }

  public double getPosition() {
    return wristMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("[Wrist] getCurrent", getCurrent());
  }
}

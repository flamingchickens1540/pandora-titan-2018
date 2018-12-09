package org.team1540.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2018.RobotMap;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.elevator.HoldElevator;
import org.team1540.rooster.wrappers.ChickenTalon;

public class Elevator extends Subsystem {

  private ChickenTalon elevatorMotorA = new ChickenTalon(RobotMap.ELEVATOR_A);
  private ChickenTalon elevatorMotorB = new ChickenTalon(RobotMap.ELEVATOR_B);

  public Elevator() {
    // add(elevatorMotorA, elevatorMotorB);
    // setPriority(10);

    elevatorMotorA.config_kP(0, Tuning.elevatorP);
    elevatorMotorA.config_kI(0, Tuning.elevatorI);
    elevatorMotorA.config_kD(0, Tuning.elevatorD);

    //

    elevatorMotorA.config_IntegralZone(0, Tuning.elevatorIZone);

    elevatorMotorA.configMotionCruiseVelocity(Tuning.elevatorCruiseVel);
    elevatorMotorA.configMotionAcceleration(Tuning.elevatorMaxAccel);

    elevatorMotorA.setInverted(true);
    elevatorMotorB.setInverted(true);

    elevatorMotorA.setBrake(true);
    elevatorMotorA.setBrake(true);

    elevatorMotorA.setSensorPhase(Tuning.isPandora);

    elevatorMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    elevatorMotorA.configAllowableClosedloopError(0, 5);
    elevatorMotorB.configAllowableClosedloopError(0, 5);
  }

  public double getCurrent() {
    return elevatorMotorA.getOutputCurrent() + elevatorMotorB.getOutputCurrent();
  }

  public int getError() {
    return elevatorMotorA.getClosedLoopError();
  }

  public double getPosition() {
    return elevatorMotorA.getSelectedSensorPosition();
  }

  public int getTrajPosition() {
    return elevatorMotorA.getActiveTrajectoryPosition();
  }

  public double getVelocity() {
    return elevatorMotorA.getSelectedSensorVelocity();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HoldElevator());
  }

  public void setMotionMagicPosition(double position) {
    elevatorMotorA.set(ControlMode.MotionMagic, position);
    elevatorMotorB.set(ControlMode.Follower, elevatorMotorA.getDeviceID());
  }

  public void set(double value) {
    elevatorMotorA.set(ControlMode.PercentOutput, value);
    elevatorMotorB.set(ControlMode.PercentOutput, value);
  }

  public void stop() {
    set(0);
  }

  public void resetEncoder() {
    elevatorMotorA.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    elevatorMotorA.config_kF(0, elevatorMotorA.getSelectedSensorVelocity()>0 ?
        Tuning.elevatorFGoingUp : Tuning.elevatorFGoingDown);
  }
}

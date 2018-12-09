package org.team1540.robot2018.testing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.rooster.adjustables.AdjustableManager;
import org.team1540.rooster.adjustables.Tunable;
import org.team1540.rooster.drive.PidDrive;
import org.team1540.rooster.drive.PidDriveFactory;
import org.team1540.rooster.wrappers.ChickenTalon;

public class DriveTestRobot extends IterativeRobot {

  // tunables
  @Tunable("Velocity Setpoint")
  public double setpoint;
  @Tunable("Invert Right Brake")
  public boolean invertRightBrake;
  @Tunable("Invert Left Brake")
  public boolean invertLeftBrake;
  @Tunable("Invert Left Output")
  public boolean invertLeftAxis;
  @Tunable("Invert Right Output")
  public boolean invertRightAxis;
  @Tunable("Invert Left Sensor")
  public boolean invertLeftSensor;
  @Tunable("Invert Right Sensor")
  public boolean invertRightSensor;
  @Tunable("Invert Left Motor")
  public boolean invertLeftMotor;
  @Tunable("Invert Right Motor")
  public boolean invertRightMotor;
  @Tunable("Max Brake Percent")
  public double maxBrakePct;
  @Tunable("Closed-Loop Ramp")
  public double closedLoopRamp;
  @Tunable("P")
  public double p = 8;
  @Tunable("I")
  public double i = 0;
  @Tunable("D")
  public double d = 0;
  @Tunable("F")
  public double f = 0;
  @Tunable("I-zone")
  public int iZone;
  @Tunable("Left shifter")
  public boolean leftPneuVal = false;
  @Tunable("Right shifter")
  public boolean rightPneuVal = false;

  // internal fields
  private PidDrive driveCmd;
  private ChickenTalon lMaster;
  private ChickenTalon lSlave1;
  private ChickenTalon lSlave2;
  private ChickenTalon rMaster;
  private ChickenTalon rSlave1;
  private ChickenTalon rSlave2;
  private Joystick joystick;

  @Override
  public void robotInit() {
    // SmartDashboard.putData(new Compressor());
    AdjustableManager.getInstance().add(this);

    joystick = new Joystick(0);
    lMaster = new ChickenTalon(1);
    lMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    lMaster.setBrake(true);
    lMaster.configClosedloopRamp(closedLoopRamp);

    lSlave1 = new ChickenTalon(2);
    lSlave1.set(ControlMode.Follower, lMaster.getDeviceID());
    lSlave1.configClosedloopRamp(closedLoopRamp);
    lSlave1.setBrake(true);

    lSlave2 = new ChickenTalon(3);
    lSlave2.set(ControlMode.Follower, lMaster.getDeviceID());
    lSlave2.configClosedloopRamp(closedLoopRamp);
    lSlave2.setBrake(true);

    rMaster = new ChickenTalon(4);
    rMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rMaster.setBrake(true);
    rMaster.configClosedloopRamp(closedLoopRamp);
    rMaster.setSensorPhase(true);
    rMaster.setInverted(true);

    rSlave1 = new ChickenTalon(5);
    rSlave1.set(ControlMode.Follower, rMaster.getDeviceID());
    rSlave1.configClosedloopRamp(closedLoopRamp);
    rSlave1.setBrake(true);

    rSlave2 = new ChickenTalon(6);
    rSlave2.set(ControlMode.Follower, rMaster.getDeviceID());
    rSlave2.configClosedloopRamp(closedLoopRamp);
    rSlave2.setBrake(true);

    driveCmd = new PidDriveFactory()
        .setSubsystem(new Subsystem() {
          @Override
          protected void initDefaultCommand() {

          }
        })
        .setLeft(lMaster)
        .setRight(rMaster)
        .setMaxVel(setpoint)
        .setJoystick(joystick)
        .setLeftAxis(1)
        .setRightAxis(5)
        .setForwardTrigger(3)
        .setBackTrigger(2)
        .setInvertLeft(invertLeftAxis)
        .setInvertRight(invertRightAxis)
        .setInvertLeftBrakeDirection(invertLeftBrake)
        .setInvertRightBrakeDirection(invertRightBrake)
        .setMaxBrakePct(maxBrakePct)
        .createPidDrive();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
    lMaster.config_kP(0, p);
    lMaster.config_kI(0, i);
    lMaster.config_kD(0, d);
    lMaster.config_kF(0, f);
    lMaster.config_IntegralZone(0, iZone);
    rMaster.config_kP(0, p);
    rMaster.config_kI(0, i);
    rMaster.config_kD(0, d);
    rMaster.config_kF(0, f);
    rMaster.config_IntegralZone(0, iZone);
    driveCmd.setMaxVel(setpoint);
    driveCmd.setJoystick(joystick);
    driveCmd.setLeftAxis(1);
    driveCmd.setRightAxis(5);
    driveCmd.setForwardTrigger(3);
    driveCmd.setBackTrigger(2);
    driveCmd.setInvertLeft(invertLeftAxis);
    driveCmd.setInvertRight(invertRightAxis);
    driveCmd.setInvertLeftBrakeDirection(invertLeftBrake);
    driveCmd.setInvertRightBrakeDirection(invertRightBrake);
    driveCmd.setMaxBrakePct(maxBrakePct);


    lMaster.configClosedloopRamp(closedLoopRamp);
    lMaster.setInverted(invertLeftMotor);
    lMaster.setSensorPhase(invertLeftSensor);

    lSlave1.configClosedloopRamp(closedLoopRamp);
    lSlave1.setInverted(invertLeftMotor);

    lSlave2.configClosedloopRamp(closedLoopRamp);
    lSlave2.setInverted(invertLeftMotor);


    rMaster.configClosedloopRamp(closedLoopRamp);
    rMaster.setInverted(invertRightMotor);
    rMaster.setSensorPhase(invertRightSensor);

    rSlave1.configClosedloopRamp(closedLoopRamp);
    rSlave1.setInverted(invertRightMotor);

    rSlave2.configClosedloopRamp(closedLoopRamp);
    rSlave2.setInverted(invertRightMotor);

    SmartDashboard.putNumber("Left Velocity", lMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", rMaster.getSelectedSensorVelocity());
  }

  @Override
  public void teleopPeriodic() {
    if (!driveCmd.isRunning()) {
      driveCmd.start();
    }
  }
}

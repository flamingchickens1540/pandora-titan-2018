package org.team1540.robot2018;

import org.team1540.base.adjustables.Tunable;

public class Tuning {

  // GENERAL
  //@Tunable("-[General] Dead Zone")
  public static double axisDeadzone = 0.1;

  //@Tunable("-[General] Manual Superstructure Control Deadzone") // Deadzone for wrist and lift
  public static double axisWristLiftDeadzone = 0.2;

  // TODO: Better method of switching tuning values globally between robots
  @Tunable("-[General] Is Pandora")
  public static boolean isPandora = false;

  // AUTO
  @Tunable("-[Auto] Stupid Drive Time")
  public static double stupidDriveTime = 2.5;

  @Tunable("-[Auto] Stupid Drive Percent")
  public static double stupidDrivePercent = -0.4;

  // INTAKE
  //@Tunable("[Intake] Auto Intake Spike Current")
  public static double intakeSpikeCurrent = 30.0;

  //@Tunable("[Intake] Auto Intake Speed Motor A")
  public static double intakeSpeedA = -1;

  //@Tunable("[Intake] Auto Intake Speed Motor B")
  public static double intakeSpeedB = -0.4;

  //@Tunable("[Intake] Auto Intake Min Time")
  public static double intakeMinTime = 1;

  //@Tunable("[Intake] Auto Intake Max Time")
  public static double intakeMaxTime = 10;

  //@Tunable("[Intake] Intake Hold Speed")
  public static double intakeHoldSpeed = -0.1;

  //@Tunable("[Intake] Eject Seconds")
  public static double intakeEjectTime = 1.0;

  //@Tunable("[Intake] Eject Speed Motor A")
  public static double intakeEjectSpeed = 0.8;

  public static double intakeEjectSpeedAuto = 0.35;

  // ARMS
  //@Tunable("[Intake] Arm Hold Speed")
  public static double armHoldSpeed = -0.2;

  //@Tunable("[Intake] Arm Drop Speed")
  public static double armDropSpeed = 0.7;

  //@Tunable("[Intake] Arm Drop Time")
  public static double armDropTime = 0.7;

  //@Tunable("[Intake] Arm Joystick Constant")
  public static double armJoystickConstant = 0.5;

  // ELEVATOR
  //@Tunable("[Elevator] kP")
  public static double elevatorP = 2;

  //@Tunable("[Elevator] kI")
  public static double elevatorI = 0.0025;

  //@Tunable("[Elevator] kD")
  public static double elevatorD = 10;

  //@Tunable("[Elevator] kF Going Up")
  public static double elevatorFGoingUp = 2.046;

  //@Tunable("[Elevator] kF Going Down")
  public static double elevatorFGoingDown = 1.2;

  //@Tunable("[Elevator] I-Zone")
  public static int elevatorIZone = 150;

  //@Tunable("[Elevator] Error Tolerance")
  public static double elevatorTolerance = 100;

  //@Tunable("[Elevator] Motion Max Acceleration")
  public static int elevatorMaxAccel = 750;

  //@Tunable("[Elevator] Motion Cruise Velocity")
  public static int elevatorCruiseVel = 475;

  //@Tunable("[Elevator] Ground Position")
  public static double elevatorGroundPosition = 5;

  //@Tunable("[Elevator] Exchange Position")
  public static double elevatorExchangePosition = 500;

  //@Tunable("[Elevator] Switch Front Position")
  public static double elevatorFrontSwitchPosition = 2900;

  //@Tunable("[Elevator] Scale Low Position")
  public static double elevatorLowScalePosition = 6200;

  //@Tunable("[Elevator] Max Position")
  public static double elevatorMaxPosition = 7400;

  //@Tunable("[Elevator] Obstacle Position")
  public static double elevatorObstaclePosition = 1300;

  //@Tunable("[Elevator] Obstacle Upper Position")
  public static double elevatorObstacleUpperPosition = 3750;

  //@Tunable("[Elevator] Max Elevator Deviation")
  public static double elevatorMaxDeviation = 200;

  //@Tunable("[Elevator] Down Multiplier")
  public static double elevatorDownMult = 0.5;

  //@Tunable("[Elevator] Max Spike Time")
  public static double elevatorSpikeTime = 0.5;

  //@Tunable("[Elevator] Max Spike Current")
  public static double elevatorCurrentThreshold = 60;

  // WRIST
  //@Tunable("[Wrist] kP")
  public static double wristP = 10.0;

  //@Tunable("[Wrist] kI")
  public static double wristI = 0;

  //@Tunable("[Wrist] kD")
  public static double wristD = 0;

  //@Tunable("[Wrist] kF")
  public static double wristF = 1.364;

  //@Tunable("[Wrist] I-Zone")
  public static int wristIzone = 0;

  //@Tunable("[Wrist] Motion Cruise Velocity")
  public static int wristCruiseVelocity = 1000;

  //@Tunable("[Wrist] Motion Max Acceleration")
  public static int wristMaxAccel = 6000;

  //@Tunable("[Wrist] Peak Current Limit")
  public static int wristCurrentLimit = 30;

  //@Tunable("[Wrist] Peak Duration")
  public static int wristPeakDuration = 0;

  //@Tunable("[Wrist] Stall Current")
  public static double wristStallCurrent = 30;

  //@Tunable("[Wrist] Stop Tolerance")
  public static double wristTolerance = 50;

  //@Tunable("[Wrist] Out Position")
  public static double wristOutPosition = 8250;

  //@Tunable("[Wrist] Back Position")
  public static double wristBackPosition = 0;

  //@Tunable("[Wrist] Transit Position")
  public static double wristTransitPosition = 3900;

  //@Tunable("[Wrist] 45 Back Position")
  public static double wrist45BackPosition = 1500;

  //@Tunable("[Wrist] 45 Forward Position")
  public static double wrist45FwdPosition = 6200;

  //@Tunable("[Wrist] Max Wrist Deviation")
  public static double maxWristDeviation = 200;

  // WINCH
  @Tunable("[Winch] In Low Velocity")
  public static double winchInLowVel = -0.4;

  @Tunable("[Winch] In High Velocity")
  public static double winchInHighVel = -1;

  // DRIVETRAIN
  //@Tunable("[Drivetrain] kP")
  public static double drivetrainVelocityP = 2;

  //@Tunable("[Drivetrain] kI")
  public static double drivetrainVelocityI = 0.001;

  //@Tunable("[Drivetrain] kD")
  public static double drivetrainVelocityD = 4;

  //@Tunable("[Drivetrain] kF")
  public static double drivetrainVelocityF = 1.2;

  //@Tunable("[Drivetrain] I-Zone")
  public static int drivetrainVelocityIZone = 100;

  // only need PD according to https://www.chiefdelphi.com/forums/showthread.php?p=1751198#post1751198
  @Tunable("[Drivetrain] Position kP")
  public static double drivetrainPositionP = 10;

  @Tunable("[Drivetrain] Position kD")
  public static double drivetrainPositionD = 0;

  //@Tunable("[Drivetrain] Braking Percent")
  public static double drivetrainBrakingPercent = 0.2;

  //@Tunable("[Drivetrain] Brake Override Thresh")
  public static double drivetrainBrakeOverrideThreshold = 0.9;

  //@Tunable("[Drivetrain] Ramp Rate")
  public static double drivetrainRampRate = 0.1;

  //@Tunable("[Drivetrain] Velocity")
  public static double drivetrainMaxVelocity = 750;

  //@Tunable("[Drivetrain] JoystickPower")
  public static double drivetrainJoystickPower = 2;

  //@Tunable("[Drivetrain] EncoderTPU")
  public static double drivetrainEncoderTPU = 53.1271477663;

  @Tunable("[MotionP] Profile Heading P")
  public static double profileHeadingP = 0;

  @Tunable("[MotionP] Profile Acceleration P")
  public static double profileAccelF = 0.0025;

  @Tunable("[MotionP] Profile Velocity P")
  public static double profileVelocityF = 0.006;

  @Tunable("[MotionP] Profile Loop Frequency")
  public static double profileLoopFrequency = 0.02;

  // AUTO
  public static double autoElevatorRaiseWait = 1;

  // CLIMBER
  @Tunable("[Climber] Trigger Full Press Threshold")
  public static double triggerFullPressThreshold = 0.9;
}

package org.team1540.robot2018;

import org.team1540.base.adjustables.Tunable;

public class Tuning {

  // GENERAL
  @Tunable("Dead Zone")
  public static double deadZone = 0.1;
  @Tunable("Is Pandora")
  public static boolean isPandora = false;

  // INTAKE
  @Tunable("Auto Intake Spike Current")
  public static double intakeSpikeCurrent = 30.0;

  @Tunable("Auto Intake Speed Motor A")
  public static double IntakeSpeedA = 1;

  @Tunable("Auto Intake Speed Motor B")
  public static double IntakeSpeedB = 0.4;

  @Tunable("Auto Intake Min Time")
  public static double intakeMinTime = 1;
  @Tunable("Auto Intake Max Time")
  public static double intakeMaxTime = 10;

  @Tunable("Intake Hold Speed")
  public static double intakeHoldSpeed = 0.1;

  @Tunable("Eject Seconds")
  public static double ejectTime = 1.0;

  @Tunable("Eject Speed Motor A")
  public static double ejectSpeedA = -1;

  @Tunable("Eject Speed Motor B")
  public static double ejectSpeedB = -1;


  // ELEVATOR
  @Tunable("Elevator P")
  public static double elevatorP = 2;

  @Tunable("Elevator I")
  public static double elevatorI = 0.0025;

  @Tunable("Elevator D")
  public static double elevatorD = 10;

  @Tunable("Elevator F Upwards")
  public static double elevatorFGoingUp = 2.5575;
  @Tunable("Elevator F Downwards")
  public static double elevatorFGoingDown = 0.75;
  @Tunable("Elevator I-Zone")
  public static int elevatorIZone = 100;

  @Tunable("Elevator Tolerance")
  public static double elevatorTolerance = 50;

  @Tunable("Elevator Max Acceleration")
  public static int elevatorMaxAccel = 300;

  @Tunable("Elevator Cruise Velocity")
  public static int elevatorCruiseVel = 400;

  // @Tunable("Elevator Ground Position")
  public static double elevatorGroundPosition = 5;

  // @Tunable("Elevator Exchange Position")
  public static double elevatorExchangePosition = 500;

  // @Tunable("Elevator Front Switch Position")
  public static double elevatorFrontSwitchPosition = 2900;

  // @Tunable("Elevator Scale Lower Position")
  public static double elevatorScalePosition = 7400;

  // @Tunable("Elevator Low Scale Position")
  public static double elevatorLowScalePosition = 6200;

  // @Tunable("Elevator Obstacle Position")
  public static double elevatorObstaclePosition = 1300;

  // @Tunable("Elevator Obstacle Upper Position")
  public static double elevatorObstacleUpperPosition = 3750;

  @Tunable("Max Elevator Deviation")
  public static double maxElevatorDeviation = 200;

  // WRIST
  @Tunable("Wrist P")
  public static double wristP = 10.0;

  @Tunable("Wrist I")
  public static double wristI = 0;

  @Tunable("Wrist D")
  public static double wristD = 0;

  @Tunable("Wrist F")
  public static double wristF = 1.364;

  @Tunable("Wrist I Zone")
  public static int wristIzone = 0;

  @Tunable("Motion Cruise Velocity")
  public static int wristCruiseVelocity = 600;

  @Tunable("WRist Max Acceleration")
  public static int wristMaxAccel = 1000;
  @Tunable("Wrist Peak Current Limit")
  public static int wristCurrentLimit;
  @Tunable("Wrist Peak Duration")
  public static int wristPeakDuration;

  @Tunable("Wrist Stop Tolerance")
  public static double wristTolerance = 50;

  // @Tunable("Wrist Out Position")
  public static double wristOutPosition = 8250;
  // @Tunable("Wrist Back Position")
  public static double wristBackPosition = 0;
  // @Tunable("Wrist Transit Position")
  public static double wristTransitPosition = 3900;
  // @Tunable("Wrist 45 Back Position")
  public static double wrist45BackPosition = 1500;
  // @Tunable("Wrist 45 Forward Position")
  public static double wrist45FwdPosition = 6200;

  @Tunable("Max wrist deviation")
  public static double maxWristDeviation = 200;

  // WINCH
  @Tunable("Winch In Speed")
  public static double winchInSpeed = 1;

  @Tunable("Winch Out Speed")
  public static double winchOutSpeed = -1; // 0.75

  // TAPE
  @Tunable("Tape In Speed")
  public static double tapeInSpeed = 0.5;

  @Tunable("Tape Out Speed")
  public static double tapeOutSpeed = -0.25;

  @Tunable("Tape Measure Multiplier")
  public static double tapeMeasureMultiplier = 1;

  @Tunable("Winch Multiplier")
  public static double winchMultiplier = 1;

  @Tunable("Climber In Speed")
  public static double climberInSpeed = 0.5;

  @Tunable("Climber Out Speed")
  public static double climberOutSpeed = -0.4;

  @Tunable("Manual Superstructure Control Deadzone")
  public static double manualControlDeadzone = 0.2;
  @Tunable("Climber In Low Speed")
  public static double climberInLowSpeed = -0.4;
  @Tunable("Climber In HIgh Speed")
  public static double climberInHighSpeed = -1;

  // DRIVETRAIN
  @Tunable("Drivetrain P")
  public static double drivetrainP = 2;

  @Tunable("Drivetrain I")
  public static double drivetrainI = 0.001;

  @Tunable("Drivetrain D")
  public static double drivetrainD = 4;

  @Tunable("Drivetrain F")
  public static double drivetrainF = 1.2;

  @Tunable("Drivetrain I-Zone")
  public static int drivetrainIZone = 100;

  public static double drivetrainBrakingPercent = 0.2;

  public static double drivetrainBrakeOverrideThreshold = 0.9;

  public static double drivetrainRampRate = 0.1;

  public static double drivetrainVelocity = 750;

  public static double drivetrainJoystickPower = 2;

  public static double drivetrainEncoderTPU = 2_000;
}

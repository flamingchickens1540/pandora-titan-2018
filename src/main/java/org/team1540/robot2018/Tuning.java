package org.team1540.robot2018;

import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
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

  // CAMERA
  //@Tunable("-[Camera] Camera ID")
  public static int camID = 0;

  // AUTO
  @Tunable("-[Auto] Stupid Drive Time")
  public static double stupidDriveTime = 2.5;

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

  // ARMS
  //@Tunable("[Intake] Arm Hold Speed")
  public static double armHoldSpeed = -0.1;

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
  public static double elevatorSpikeTime = 0;

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
  public static int wristCruiseVelocity = 600;

  //@Tunable("[Wrist] Motion Max Acceleration")
  public static int wristMaxAccel = 1000;

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

  // DRIVETRAIN
  //@Tunable("[Drivetrain] kP")
  public static double drivetrainP = 2;

  //@Tunable("[Drivetrain] kI")
  public static double drivetrainI = 0.001;

  //@Tunable("[Drivetrain] kD")
  public static double drivetrainD = 4;

  //@Tunable("[Drivetrain] kF")
  public static double drivetrainF = 1.2;

  //@Tunable("[Drivetrain] I-Zone")
  public static int drivetrainIZone = 100;

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

  @Tunable("[MotionP] MaxVelocity")
  public static double profileMaxVel = 40;

  @Tunable("[MotionP] MaxAcceleration")
  public static double profileMaxAccel = 40;

  @Tunable("[MotionP] MaxJerk")
  public static double profileMaxJerk = 2300;

  //@Tunable("[MotionP] SecondsFromNeutralToFull")
  public static double profileSecondsFromNeutralToFull = 0;

  //@Tunable("[MotionP] SampleRate")
  public static int profileSampleRate = Config.SAMPLES_FAST;

  //@Tunable("[MotionP] TimeStep")
  public static double profileTimeStep = 0.05;

  //@Tunable("[MotionP] FitMethod")
  public static FitMethod profileFitMethod = FitMethod.HERMITE_CUBIC;

  //@Tunable("[MotionP] LeftEncoderTicksPerUnit")
  public static double profileLeftTPU = 52;

  //@Tunable("[MotionP] RightEncoderTicksPerUnit")
  public static double profileRightTPU = 52;

  @Tunable("[MotionP] WheelbaseWidth")
  public static double profileBaseWidth = 24;

  // TODO: Rename profileWheelDistance to be more specific
  //@Tunable("[MotionP] distanceBetweenWheels")
  public static double profileWheelDistance = 11.812;

  @Tunable("[MotionP] Profile Heading P")
  public static double profileHeadingP = 100;

  @Tunable("[MotionP] Profile Acceleration P")
  public static double profileAccelP = 0;

  @Tunable("[MotionP] Profile P")
  public static double profileP = 0.5;
  @Tunable("[MotionP] Profile D")
  public static double profileD;

  @Tunable("[MotionP] Profile Loop Frequency")
  public static double profileLoopFrequency = 0.01;

  @Tunable("[MotionP] Profile Test X")
  public static double profileTestX = 50;

  @Tunable("[MotionP] Profile Test Y")
  public static double profileTestY = 0;

  @Tunable("[MotionP] Profile Test Angle")
  public static double profileTestAngle = 0;
}

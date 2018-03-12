package org.team1540.robot2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Waypoint;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;
import org.opencv.core.Mat;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.power.PowerManager;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.commands.TankDrive;
import org.team1540.robot2018.commands.auto.AutonomousProfiling;
import org.team1540.robot2018.commands.auto.AutonomousProfiling.TrajectorySegment;
import org.team1540.robot2018.commands.auto.DriveTimed;
import org.team1540.robot2018.commands.elevator.JoystickElevator;
import org.team1540.robot2018.commands.elevator.MoveElevatorSafe;
import org.team1540.robot2018.commands.groups.GroundPosition;
import org.team1540.robot2018.commands.intake.Eject;
import org.team1540.robot2018.commands.wrist.CalibrateWrist;
import org.team1540.robot2018.commands.wrist.JoystickWrist;
import org.team1540.robot2018.commands.wrist.MoveWrist;
import org.team1540.robot2018.subsystems.Arms;
import org.team1540.robot2018.subsystems.ClimberWinch;
import org.team1540.robot2018.subsystems.DriveTrain;
import org.team1540.robot2018.subsystems.Elevator;
import org.team1540.robot2018.subsystems.Intake;
import org.team1540.robot2018.subsystems.Wrist;

public class Robot extends IterativeRobot {
  public static final DriveTrain drivetrain = new DriveTrain();
  public static final Intake intake = new Intake();
  public static final Arms arms = new Arms();
  public static final Elevator elevator = new Elevator();
  public static final Wrist wrist = new Wrist();
  public static final ClimberWinch winch = new ClimberWinch();

  private Command emergencyDriveCommand = new TankDrive();
  private Command dumbElevator = new JoystickElevator();
  private Command dumbWrist = new JoystickWrist();

  private SendableChooser<String> autoPosition;
  private SendableChooser<Boolean> driveMode;

  private SendableChooser<Boolean> elevatorMoveMode;
  private SendableChooser<Boolean> wristMoveMode;

  private Command autoCommand;

  @Override
  public void robotInit() {
    // disable unused things
    LiveWindow.disableAllTelemetry();
    PowerManager.getInstance().interrupt();


    elevatorMoveMode = new SendableChooser<>();
    elevatorMoveMode.addDefault("PID Elevator", true);
    elevatorMoveMode.addObject("Manual Override", false);

    wristMoveMode = new SendableChooser<>();
    wristMoveMode.addDefault("PID Wrist", true);
    wristMoveMode.addObject("Manual Override", false);


    // TODO: Move auto chooser into command
    AdjustableManager.getInstance().add(new Tuning());
    autoPosition = new SendableChooser<>();
    autoPosition.addObject("Middle", "Middle");
    autoPosition.addObject("Left", "Left");
    autoPosition.addObject("Right", "Right");
    autoPosition.addDefault("Right Hook", "Right Hook");
    autoPosition.addObject("Stupid", "Stupid");

    SmartDashboard.putData("Auto mode", autoPosition);

    driveMode = new SendableChooser<>();
    driveMode.addDefault("PID Drive", false);
    driveMode.addObject("Manual Override", true);
    SmartDashboard.putData("[Drivetrain] ***** DRIVE OVERRIDE *****", driveMode);

    // TODO: Move SmartDashboard commands to separate class
    Command zeroWrist = new SimpleCommand("[Wrist] Zero Wrist", wrist::resetEncoder);
    zeroWrist.setRunWhenDisabled(true);
    SmartDashboard.putData(zeroWrist);

    Command zeroElevator = new SimpleCommand("[Elevator] Zero Elevator", elevator::resetEncoder);
    zeroElevator.setRunWhenDisabled(true);
    SmartDashboard.putData(zeroElevator);

    // TODO: Move camera crosshairs into separate (command?)
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(Tuning.camID);
      camera.setResolution(320, 240);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo(
          "Camera " + Tuning.camID, 320, 240);

      Mat source = new Mat();
      Mat output = new Mat();

      while (!Thread.interrupted()) {
        cvSink.grabFrame(source);
        // Point pt1 = new Point(source.width() / 2 + Tuning.crosshairsSize, source.height() / 2);
        // Point pt2 = new Point(source.width() / 2 - Tuning.crosshairsSize, source.height() / 2);
        // Point pt3 = new Point(source.width() / 2, source.height() / 2 + Tuning.crosshairsSize);
        // Point pt4 = new Point(source.width() / 2, source.height() / 2 - Tuning.crosshairsSize);
        // Imgproc.line(source, pt1, pt2, new Scalar(0, 255, 0), Tuning.crosshairsThicccness);
        // Imgproc.line(source, pt3, pt4, new Scalar(0, 255, 0), Tuning.crosshairsThicccness);
        outputStream.putFrame(source);
      }
    }).start();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void autonomousInit() {
    // TODO: Move auto logic into command
    elevator.resetEncoder();
    switch (autoPosition.getSelected()) {
      case "Left":
        System.out.println("Left Auto Selected");
        autoCommand = new CommandGroup() {
          {
            if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
              System.out.println("Going for Left Switch");
              addSequential(new AutonomousProfiling(new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(120, 50, 0), false)));
              addSequential(new MoveWrist(Tuning.wrist45BackPosition));
              addSequential(new Eject(1));
            } else {
              System.out.println("Just Crossing the Line");
              addSequential(new AutonomousProfiling(new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(134, 0, 0), false))); // go straight
            }
            addSequential(new CalibrateWrist());
          }
        };
        break;

      case "Middle":
        System.out.println("Middle Auto Selected");
        autoCommand = new CommandGroup() {
          {
            if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
              System.out.println("Going for Left Switch");
              addSequential(new AutonomousProfiling(new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(102, -123, 0), false)));
              addSequential(new MoveWrist(Tuning.wrist45BackPosition));
              addSequential(new Eject(1));
            } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
              System.out.println("Going for Right Switch");
              addSequential(new AutonomousProfiling(new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(106, 85, 0), false)));
              addSequential(new MoveWrist(Tuning.wrist45BackPosition));
              addSequential(new Eject(1));
            } else {
              DriverStation.reportError(
                  "Match data could not get owned switch side, reverting to base auto",
                  false);
              addSequential(new DriveTimed(ControlMode.PercentOutput, Tuning.stupidDriveTime, -0.4));
            }
            addSequential(new CalibrateWrist());
          }
        };
        break;

      case "Right":
        System.out.println("Right Auto Selected");
        autoCommand = new CommandGroup() {
          {
            addSequential(new AutonomousProfiling(new TrajectorySegment(
                new Waypoint(0, 0, 0),
                new Waypoint(134, 0, 0), false)));
            if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
              System.out.println("Going for Right Switch");
              addSequential(new MoveWrist(Tuning.wrist45BackPosition));
              addSequential(new Eject(1));
            }
            addSequential(new CalibrateWrist());
          }
        };
        break;

      case "Right Hook":
        System.out.println("Right Hook Selected");
        autoCommand = new CommandGroup() {
          {
            if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
              addSequential(new AutonomousProfiling(80, new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(284, 0, 0), false)));
              System.out.println("Going for Right Scale");
              addParallel(new MoveWrist(Tuning.wristTransitPosition));
              addSequential(new DriveTimed(ControlMode.Velocity, 0.8, -450, 150));
              addSequential(new MoveElevatorSafe(false, Tuning.elevatorMaxPosition));
              addSequential(new MoveWrist(Tuning.wristBackPosition));
              addSequential(new Eject(0.6));
            } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
              addSequential(new AutonomousProfiling(new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(114, 0, 0), false)));
              System.out.println("Going for Right Switch");
              addSequential(new DriveTimed(ControlMode.PercentOutput, 1.6, -0.6, 0.1));
              addSequential(new MoveWrist(Tuning.wrist45BackPosition));
              addSequential(new Eject(1));
            } else {
              addSequential(new AutonomousProfiling(new TrajectorySegment(
                  new Waypoint(0, 0, 0),
                  new Waypoint(114, 0, 0), false)));
            }
            addSequential(new CalibrateWrist());
            addSequential(new GroundPosition());
          }
        };
        break;

      case "Stupid":
        System.out.println("Stupid Auto Selected");
        autoCommand = new CommandGroup() {
          {
            addSequential(new DriveTimed(ControlMode.PercentOutput, Tuning.stupidDriveTime, -0.4));
            addSequential(new CalibrateWrist());
          }
        };
        break;
    }

    autoCommand.start();
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    // TODO: Add a command chooser to ROOSTER
    // for drive override
    if (driveMode.getSelected()) {
      // oh no encoders broke
      emergencyDriveCommand.start();
    } else {
      emergencyDriveCommand.cancel();
    }

    if (elevatorMoveMode.getSelected()) {
      dumbElevator.start();
    } else {
      dumbElevator.cancel();
    }

    if (wristMoveMode.getSelected()) {
      //:(){ :|:& };:
      dumbWrist.start();
    } else {
      dumbWrist.cancel();
    }
  }
}

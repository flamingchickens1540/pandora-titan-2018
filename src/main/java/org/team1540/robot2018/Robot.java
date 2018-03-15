package org.team1540.robot2018;

import static org.team1540.robot2018.Tuning.profileTimeStep;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import java.io.File;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;
import org.opencv.core.Mat;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.power.PowerManager;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.commands.TankDrive;
import org.team1540.robot2018.commands.auto.DriveTimed;
import org.team1540.robot2018.commands.auto.sequences.RightHookAuto;
import org.team1540.robot2018.commands.auto.sequences.RightScaleAuto;
import org.team1540.robot2018.commands.auto.sequences.SimpleProfileAuto;
import org.team1540.robot2018.commands.auto.sequences.SingleCubeSwitchAuto;
import org.team1540.robot2018.motion.FollowProfile;
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
  public static CSVProfileManager profiles;
  public static AHRS gyro = new AHRS(Port.kMXP);

  private Command emergencyDriveCommand = new TankDrive();

  private SendableChooser<String> autoPosition;
  private SendableChooser<Boolean> driveMode;

  private Command autoCommand;

  @Override
  public void robotInit() {
    // disable unused things
    LiveWindow.disableAllTelemetry();
    PowerManager.getInstance().interrupt();

    // TODO: Move auto chooser into command
    AdjustableManager.getInstance().add(new Tuning());
    autoPosition = new SendableChooser<>();
    autoPosition.addObject("Middle", "Middle");
    autoPosition.addObject("Left", "Left");
    autoPosition.addObject("Right", "Right");
    autoPosition.addDefault("Right Hook", "Right Hook");
    autoPosition.addObject("Stupid", "Stupid");
    autoPosition.addObject("Advanced Follower Test", "Advanced Follower Test");

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


    // initialize profiles
    // unlike other static fields, initialized here because there's a high likelihood of it throwing
    // an exception and exceptions thrown during static initialization are not fun.
    profiles = new CSVProfileManager(new File("/home/lvuser/profiles"));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void autonomousInit() {
    // TODO: Move auto logic into command
    elevator.resetEncoder();
    wrist.setSensorPosition(0);
    switch (autoPosition.getSelected()) {
      case "Left":
        System.out.println("Left Auto Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
          System.out.println("Going for Left Switch");
          autoCommand = new SingleCubeSwitchAuto("left_to_left_switch");
        } else {
          System.out.println("Just Crossing the Line");
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;

      case "Middle":
        System.out.println("Middle Auto Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
          System.out.println("Going for Left Switch");
          autoCommand = new SingleCubeSwitchAuto("middle_to_left_switch");
        } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
          System.out.println("Going for Right Switch");
          autoCommand = new SingleCubeSwitchAuto("middle_to_right_switch");
        } else {
          DriverStation.reportError(
              "Match data could not get owned switch side, reverting to base auto",
              false);
          autoCommand = new DriveTimed(ControlMode.PercentOutput, Tuning.stupidDriveTime, -0.4);
        }
        break;

      case "Right":
        System.out.println("Right Auto Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
          autoCommand = new SingleCubeSwitchAuto("go_straight");
        } else {
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;

      case "Right Hook":
        System.out.println("Right Hook Selected");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          System.out.println("Going for scale");
          autoCommand = new RightScaleAuto();
        } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
          System.out.println("Going for switch");
          autoCommand = new RightHookAuto();
        } else {
          System.out.println("Crossing the line");
          autoCommand = new SimpleProfileAuto("right_hook_approach");
        }
        break;

      case "Stupid":
        System.out.println("Stupid Auto Selected");
        autoCommand = new DriveTimed(ControlMode.PercentOutput, Tuning.stupidDriveTime, 0.4);
        break;

      case "Advanced Follower Test":
        System.out.println("Testing Advanced Auto");
        // DriveProfile profile = profiles.getProfile("go_straight");
        double turningRadius = Math.sqrt(Math.pow(Tuning.profileBaseWidth, 2) + Math.pow
            (Tuning.profileWheelDistance, 2));
        Config config = new Config(Tuning.profileFitMethod, Tuning.profileSampleRate, profileTimeStep,
            Tuning.profileMaxVel, Tuning.profileMaxAccel, Tuning.profileMaxJerk);
        Trajectory trajectory = Pathfinder.generate(new Waypoint[]{new Waypoint(0, 0, 0), new Waypoint(Tuning.profileTestDistance, 0, 0)}, config);
        TankModifier modifier = new TankModifier(trajectory).modify(turningRadius);
        Trajectory left = modifier.getLeftTrajectory();
        Trajectory right = modifier.getRightTrajectory();

        Robot.drivetrain.zeroEncoders();
        autoCommand = new FollowProfile(left, right);
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
    SmartDashboard.putNumber("Elevator postion", elevator.getPosition());
    double leftDistance = drivetrain.getLeftPosition();
    double rightDistance = -drivetrain.getRightPosition();

    SmartDashboard.putNumber("Left Distance", leftDistance);
    SmartDashboard.putNumber("Right Distance", rightDistance);
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
  }
}

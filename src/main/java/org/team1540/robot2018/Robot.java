package org.team1540.robot2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.power.PowerManager;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.commands.TankDrive;
import org.team1540.robot2018.commands.auto.DriveTimed;
import org.team1540.robot2018.commands.auto.sequences.ProfileDoubleScaleAuto;
import org.team1540.robot2018.commands.auto.sequences.ProfileScaleAuto;
import org.team1540.robot2018.commands.auto.sequences.SimpleProfileAuto;
import org.team1540.robot2018.commands.auto.sequences.SingleCubeSwitchAuto;
import org.team1540.robot2018.commands.auto.sequences.SwitchDoubleCube;
import org.team1540.robot2018.subsystems.Arms;
import org.team1540.robot2018.subsystems.ClimberTape;
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
  public static final ClimberTape tape = new ClimberTape();
  public static CSVProfileManager profiles;
  public static AHRS navx = new AHRS(Port.kMXP);

  private Command emergencyDriveCommand = new TankDrive();

  private SendableChooser<String> autoPosition;
  private SendableChooser<Boolean> driveMode;

  private Command autoCommand;

  PowerDistributionPanel pdp = new PowerDistributionPanel();

  @Override
  public void robotInit() {
    // disable unused things
    LiveWindow.disableAllTelemetry();
    PowerManager.getInstance().interrupt();
    // PowerManager.getInstance().setUpdateDelay(40);

    // TODO: Move auto chooser into command
    AdjustableManager.getInstance().add(new Tuning());
    autoPosition = new SendableChooser<>();
    autoPosition.addDefault("Middle", "Middle");
    autoPosition.addObject("Left Scale Then Switch", "Left Scale Then Switch");
    autoPosition.addObject("Left Double Scale Then Switch", "Left Double Scale Then Switch");
    autoPosition.addObject("Left Scale No Switch", "Left Scale No Switch");
    autoPosition.addObject("Left Scale Hook No Switch", "Left Scale Hook No Switch");
    autoPosition.addObject("Left Hook Switch Then Scale", "Left Hook Switch Then Scale");
    autoPosition.addObject("Left Hook Switch Then Double Scale", "Left Hook Switch Then Double Scale");
    autoPosition.addObject("Right Scale Then Switch", "Right Scale Then Switch");
    // autoPosition.addObject("Right Scale No Switch", "Right Scale No Switch");
    autoPosition.addObject("Left Hook Switch No Scale", "Left Hook Switch No Scale");
    autoPosition.addObject("Center Double Cube", "Center Double Cube");
    autoPosition.addObject("Right Hook", "Right Hook Switch");
    autoPosition.addObject("Cross Line", "Cross Line");
    autoPosition.addObject("Stupid", "Stupid");
    autoPosition.addObject("Do Nothing", "Do Nothing");

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

    Command zeroDrivetrain = new SimpleCommand("[Drivetrain] Zero Drivetrain", drivetrain::zeroEncoders);
    zeroDrivetrain.setRunWhenDisabled(true);
    SmartDashboard.putData(zeroDrivetrain);

    UsbCamera overheadCam = CameraServer.getInstance().startAutomaticCapture(Tuning.overheadCamID);
    overheadCam.setResolution(128, 73);
    overheadCam.setFPS(30);

    UsbCamera turretCam = CameraServer.getInstance().startAutomaticCapture(Tuning.turretCamID);
    turretCam.setResolution(128, 73);
    overheadCam.setFPS(30);

    // Motor Burnout Warning
    new Thread(() -> {
      int rightBurnout = isBurnedOut(5, 25, new ArrayList<>(Arrays.asList(1, 2)));
      System.out.println(rightBurnout + " other " + pdp.getCurrent(2));
      // int leftBurnout = isBurnedOut(5, 25, new Integer[]{13, 14, 15});
      // if (rightBurnout + leftBurnout != -2) {
      //   System.out.println("Motor "+leftBurnout+" or "+rightBurnout+" is burned out");
      // }
    }).start();


    Command refreshProfiles = new SimpleCommand("[MotionP] Refresh Motion Profiles",
        () -> profiles = new CSVProfileManager(new File("/home/lvuser/profiles")));
    refreshProfiles.setRunWhenDisabled(true);
    SmartDashboard.putData(refreshProfiles);

    // initialize profiles
    // unlike other static fields, initialized here because there's a high likelihood of it throwing
    // an exception and exceptions thrown during static initialization are not fun.
    profiles = new CSVProfileManager(new File("/home/lvuser/profiles"));

    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(PowerManager.getInstance());
  }

  private int isBurnedOut(double burnoutLimit, double otherLimit, List<Integer> pdpIndicies) {
    for (Integer pdpIndicy : pdpIndicies) {
      double averageOtherCurrent =
          pdp.getCurrent(pdpIndicies.get(0)) + pdp.getCurrent(pdpIndicies.get(1)) / 2;
      if (averageOtherCurrent > otherLimit && pdp.getCurrent(pdpIndicies.get(3)) < burnoutLimit) {
        return pdpIndicies.get(3);
      }
      pdpIndicies.add(pdpIndicies.remove(pdpIndicies.size() - 1));
    }
    return -1;
  }

  @Override
  public void disabledInit() {
    Robot.drivetrain.configTalonsForVelocity();
    Robot.drivetrain.setBrake(false);
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @SuppressWarnings("Duplicates")
  @Override
  public void autonomousInit() {
    // TODO: Move auto logic into command
    // PowerManager.getInstance().setRunning(false);
    elevator.resetEncoder();
    wrist.setSensorPosition(0);
    Robot.drivetrain.setBrake(true);
    autoCommand = null;
    switch (autoPosition.getSelected()) {
      case "Left Double Scale Then Switch":
        System.out.println("Left Scale Auto Selected (Then Switch)");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
          System.out.println("Going for Left Scale");
          autoCommand = new ProfileDoubleScaleAuto("left_scale_straight", "left_scale_straight_back_to_switch", "left_switch_straight_back_to_scale");
        } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          // TODO: crossover
          if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
            System.out.println("Going for Left Switch");
            autoCommand = new SingleCubeSwitchAuto("left_hook");
          } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
            System.out.println("Just Crossing the Line");
            autoCommand = new SimpleProfileAuto("go_straight");
          } else {
            DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
            autoCommand = new SimpleProfileAuto("go_straight");
          }
        } else {
          DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;
      case "Left Scale Then Switch":
        System.out.println("Left Scale Auto Selected (Then Switch)");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
          System.out.println("Going for Left Scale");
          autoCommand = new ProfileScaleAuto("left_scale");
        } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          // TODO: crossover
          if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
            System.out.println("Going for Left Switch");
            autoCommand = new SingleCubeSwitchAuto("left_hook");
          } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
            System.out.println("Just Crossing the Line");
            autoCommand = new SimpleProfileAuto("go_straight");
          } else {
            DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
            autoCommand = new SimpleProfileAuto("go_straight");
          }
        } else {
          DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;
      case "Left Scale No Switch":
        System.out.println("Left Scale Auto Selected (No Switch)");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
          System.out.println("Going for Left Scale");
          autoCommand = new ProfileDoubleScaleAuto("left_scale", "left_scale_back_to_switch", "left_switch_back_to_scale");
        } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          System.out.println("Just Crossing the Line");
          autoCommand = new SimpleProfileAuto("go_straight");
        } else {
          DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;
      case "Left Scale Hook No Switch":
        System.out.println("Left Scale Hook No Switch");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
          System.out.println("Going for Left Scale Hook");
          autoCommand = new ProfileScaleAuto("left_scale_hook");
        } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          // TODO: crossover
          if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
            System.out.println("Going for Left Switch");
            autoCommand = new SingleCubeSwitchAuto("left_hook");
          } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
            System.out.println("Just Crossing the Line");
            autoCommand = new SimpleProfileAuto("go_straight");
          } else {
            DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
            autoCommand = new SimpleProfileAuto("go_straight");
          }
        } else {
          DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;
      case "Left Double Scale No Switch":
        System.out.println("Left Scale Auto Selected (No Switch)");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
          System.out.println("Going for Left Scale");
          autoCommand = new ProfileDoubleScaleAuto("left_scale_straight", "left_scale_straight_back_to_switch", "left_switch_straight_back_to_scale");
        } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          System.out.println("Just Crossing the Line");
          autoCommand = new SimpleProfileAuto("go_straight");
        } else {
          DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;

      case "Left Hook Switch Then Double Scale":
        System.out.println("Left Hook Switch Then Scale Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
          System.out.println("Going for left switch");
          autoCommand = new SingleCubeSwitchAuto("left_hook");
        } else {
          System.out.println("Going for scale");
          if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
            autoCommand = new ProfileDoubleScaleAuto("left_scale_straight", "left_scale_straight_back_to_switch", "left_switch_straight_back_to_scale");
          } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
            System.out.println("Just Crossing the Line");
            autoCommand = new SimpleProfileAuto("go_straight"); // TODO: crossover
          } else {
            DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
            autoCommand = new SimpleProfileAuto("go_straight");
          }
        }
        break;
      case "Left Hook Switch Then Scale":
        System.out.println("Left Hook Switch Then Scale Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
          System.out.println("Going for left switch");
          autoCommand = new SingleCubeSwitchAuto("left_hook");
        } else {
          System.out.println("Going for scale");
          if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
            autoCommand = new ProfileDoubleScaleAuto("left_scale_straight", "left_scale_straight_back_to_switch", "left_switch_straight_back_to_scale");
          } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
            System.out.println("Just Crossing the Line");
            autoCommand = new SimpleProfileAuto("go_straight"); // TODO: crossover
          } else {
            DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
            autoCommand = new SimpleProfileAuto("go_straight");
          }
        }
        break;

      case "Left Hook Switch No Scale":
        System.out.println("Left Hook Switch No Scale Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
          System.out.println("Going for left switch");
          autoCommand = new SingleCubeSwitchAuto("left_hook");
        } else {
          System.out.println("Just Crossing the Line");
          autoCommand = new SimpleProfileAuto("go_straight"); // TODO: crossover
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
      case "Center Double Cube":
        System.out.println("Middle Auto Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
          System.out.println("Going for Left Switch");
          autoCommand = new SwitchDoubleCube("left");
        } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
          System.out.println("Going for Right Switch");
          autoCommand = new SwitchDoubleCube("right");
        }
        break;

      case "Right Hook Switch":
        System.out.println("Right Hook Switch Selected");
        if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
          System.out.println("Going for the right switch");
          autoCommand = new SingleCubeSwitchAuto("right_hook"); //TODO: make this
        } else {
          System.out.println("Crossing the line");
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;
      case "Right Scale Then Switch":
        System.out.println("Right Scale Auto Selected (Then Switch)");
        if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.RIGHT) {
          System.out.println("Going for Right Scale");
          autoCommand = new ProfileScaleAuto("right_scale");
        } else if (MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT) {
          // TODO: crossover
          if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT) {
            System.out.println("Going for Right Switch");
            autoCommand = new SingleCubeSwitchAuto("right_hook");
          } else if (MatchData.getOwnedSide(GameFeature.SWITCH_NEAR) == OwnedSide.LEFT) {
            System.out.println("Just Crossing the Line");
            autoCommand = new SimpleProfileAuto("go_straight");
          } else {
            DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
            autoCommand = new SimpleProfileAuto("go_straight");
          }
        } else {
          DriverStation.reportError("Match data could not get owned scale side, reverting to base auto", false);
          autoCommand = new SimpleProfileAuto("go_straight");
        }
        break;

      case "Cross Line":
        autoCommand = new SimpleProfileAuto("go_straight");
        break;

      case "Stupid":
        System.out.println("Stupid Auto Selected");
        autoCommand = new DriveTimed(ControlMode.PercentOutput, Tuning.stupidDriveTime, Tuning.stupidDrivePercent);
        break;
    }

    if (autoCommand != null) {
      autoCommand.start();
    }
  }

  @Override
  public void teleopInit() {
    // PowerManager.getInstance().setRunning(true);
    Robot.drivetrain.setBrake(true);
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    Robot.drivetrain.configTalonsForVelocity();
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

    SmartDashboard.putNumber("Gyro Angle", Robot.navx.getAngle());
    SmartDashboard.putData(Scheduler.getInstance());
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

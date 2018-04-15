package org.team1540.robot2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.power.PowerManager;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.commands.TankDrive;
import org.team1540.robot2018.commands.auto.DriveTimed;
import org.team1540.robot2018.commands.auto.sequences.ProfileMultiSwitchAuto;
import org.team1540.robot2018.commands.auto.sequences.ProfileScaleAuto;
import org.team1540.robot2018.commands.auto.sequences.ProfileSwitchScaleAuto;
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

  private SendableChooser<AutoMode> autoPosition;
  private SendableChooser<Boolean> driveMode;

  private Command autoCommand;

  private static final BooleanSupplier SCALE_OWNED_LEFT = () -> (
      MatchData.getOwnedSide(GameFeature.SCALE) == OwnedSide.LEFT);
  private static final BooleanSupplier SCALE_OWNED_RIGHT = () -> (MatchData.getOwnedSide
      (GameFeature.SCALE) == OwnedSide.RIGHT);
  private static final BooleanSupplier SWITCH_OWNED_LEFT = () -> (MatchData.getOwnedSide
      (GameFeature.SWITCH_NEAR) == OwnedSide.LEFT);
  private static final BooleanSupplier SWITCH_OWNED_RIGHT = () -> (MatchData.getOwnedSide
      (GameFeature.SWITCH_NEAR) == OwnedSide.RIGHT);
  private static final BooleanSupplier SCALE_NO_DATA = () -> (MatchData.getOwnedSide(GameFeature
      .SCALE) == OwnedSide.UNKNOWN);
  private static final BooleanSupplier SWITCH_NO_DATA = () -> (MatchData.getOwnedSide(GameFeature
      .SWITCH_NEAR) == OwnedSide.UNKNOWN);

  private static final Message SCALE_NO_DATA_MESSAGE = new Message("Could not get scale data", true);
  private static final Message SWITCH_NO_DATA_MESSAGE = new Message("Could not get switch data", true);

  @Override
  public void robotInit() {
    // disable unused things
    LiveWindow.disableAllTelemetry();
    PowerManager.getInstance().interrupt();
    // PowerManager.getInstance().setUpdateDelay(40);

    // TODO: Move auto chooser into command
    AdjustableManager.getInstance().add(new Tuning());

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

    Command zeroNavX = new SimpleCommand("[Gyro] Zero Gyro", navx::zeroYaw);
    zeroNavX.setRunWhenDisabled(true);
    SmartDashboard.putData(zeroNavX);


    // UsbCamera overheadCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.OVERHEAD_CAM_ID);
    // overheadCam.setResolution(128, 73);
    // overheadCam.setFPS(30);
    //
    // UsbCamera turretCam = CameraServer.getInstance().startAutomaticCapture(RobotMap.TURRET_CAM_ID);
    // turretCam.setResolution(128, 73);
    // overheadCam.setFPS(30);


    Command refreshProfiles = new SimpleCommand("[MotionP] Refresh Motion Profiles",
        () -> profiles = new CSVProfileManager(new File("/home/lvuser/profiles")));
    refreshProfiles.setRunWhenDisabled(true);
    SmartDashboard.putData(refreshProfiles);

    // initialize profiles
    // unlike other static fields, initialized here because there's a high likelihood of it throwing
    // an exception and exceptions thrown during static initialization are not fun.
    profiles = new CSVProfileManager(new File("/home/lvuser/profiles"));

    autoPosition = new SendableChooser<>();
    for (AutoMode autoMode : AutoMode.values()) {
      if (autoMode.isDefault) {
        autoPosition.addDefault(autoMode.name, autoMode);
      } else {
        autoPosition.addObject(autoMode.name, autoMode);
      }
    }
    SmartDashboard.putData("Auto mode", autoPosition);
  }

  @Override
  public void disabledInit() {
    Robot.drivetrain.configTalonsForVelocity();
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void autonomousInit() {
    // TODO: Move auto logic into command
    // PowerManager.getInstance().setRunning(false);
    elevator.resetEncoder();
    wrist.setSensorPosition(0);
    Optional<Command> autoCommandOptional = findCommand(autoPosition.getSelected().root);
    if (autoCommandOptional.isPresent()) {
      autoCommand = autoCommandOptional.get();
      autoCommand.start();
    } else {
      DriverStation.reportError("Could not find auto command", false);
    }
  }

  // Depth first search
  private Optional<Command> findCommand(DecisionNode root) {
    if (root.condition.getAsBoolean()) {
      if (root.getProfile().isPresent()) {
        root.getMessage().ifPresent(Message::displayMessage);
        return Optional.of(root.getProfile().get().autoCommand.get());
      } else {
        if (root.getChildren().isPresent()) {
          for (DecisionNode node : root.getChildren().get()) {
            Optional<Command> command = findCommand(node);
            if (command.isPresent()) {
              return command;
            }
          }
        }
      }
    }
    return Optional.empty();
  }

  @Override
  public void teleopInit() {
    // PowerManager.getInstance().setRunning(true);
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

  /**
   * Set of all available autonomous modes. To create a new autonomous mode, create a new enum and
   * identify the root {@link DecisionNode} from which to perform a depth first tree traversal in
   * search of a leaf.
   */
  private enum AutoMode {

    //     new DecisionNode(SCALE_OWNED_RIGHT,
    //     new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SCALE_NO_DATA_MESSAGE))),
    //         new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE)),
    //         new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.GO_STRAIGHT),
    //         new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.LEFT_HOOK),
    // LEFT_DOUBLE_SCALE_THEN_SWITCH(new DecisionNode(new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.LEFT_DOUBLE_SCALE_THEN_SWITCH),
    LEFT_SCALE_NO_SWTICH(new DecisionNode(
        new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.LEFT_SCALE),
        new DecisionNode(SCALE_OWNED_RIGHT, AutonomousRoutine.GO_STRAIGHT),
        new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SCALE_NO_DATA_MESSAGE))),
    LEFT_SCALE_THEN_SWITCH(new DecisionNode(
        new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.LEFT_SCALE),
        new DecisionNode(SCALE_OWNED_RIGHT,
            new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.LEFT_HOOK),
            new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.GO_STRAIGHT),
            new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE)),
        new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SCALE_NO_DATA_MESSAGE))),
    // LEFT_DOUBLE_SCALE_NO_SWITCH(new DecisionNode(new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.LEFT_SCALE_NO_SWITCH),
    //     new DecisionNode(SCALE_OWNED_RIGHT, AutonomousRoutine.GO_STRAIGHT),
    //     new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SCALE_NO_DATA_MESSAGE))),
    // LEFT_HOOK_SWITCH_THEN_DOUBLE_SCALE(new DecisionNode(new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.LEFT_HOOK),
    //     new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.LEFT_SCALE_STRAIGHT),
    //     new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE),
    //     new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SCALE_NO_DATA_MESSAGE))),
    LEFT_SWITCH_THEN_SCALE(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.LEFT_HOOK),
        new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.LEFT_SCALE),
        new DecisionNode(SCALE_OWNED_RIGHT, AutonomousRoutine.GO_STRAIGHT),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE),
        new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SCALE_NO_DATA_MESSAGE))),
    LEFT_SWITCH_NO_SCALE(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.LEFT_HOOK),
        new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.GO_STRAIGHT),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE)
    )),
    MIDDLE(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.MIDDLE_LEFT_SWITCH_1CUBE),
        new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.MIDDLE_RIGHT_SWITCH_1CUBE),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE))),
    MIDDLE_2_CUBE(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.SWITCH_DOUBLE_CUBE_LEFT),
        new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.SWITCH_DOUBLE_CUBE_RIGHT),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE))),
    MIDDLE_MULTI_CUBE(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.SWITCH_DOUBLE_CUBE_LEFT),
        new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.SWITCH_DOUBLE_CUBE_RIGHT),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE))),
    MIDDLE_SWITCH_SCALE(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_LEFT, new DecisionNode(
            new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.MIDDLE_LEFT_SWITCH_SCALE),
            new DecisionNode(SCALE_OWNED_RIGHT, AutonomousRoutine.MIDDLE_LEFT_SWITCH_MULTI),
            new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.MIDDLE_LEFT_SWITCH_MULTI).setMessage(SCALE_NO_DATA_MESSAGE)
        )),
        new DecisionNode(SWITCH_OWNED_RIGHT, new DecisionNode(
            new DecisionNode(SCALE_OWNED_RIGHT, AutonomousRoutine.MIDDLE_RIGHT_SWITCH_SCALE),
            new DecisionNode(SCALE_OWNED_LEFT, AutonomousRoutine.MIDDLE_RIGHT_SWITCH_MULTI),
            new DecisionNode(SCALE_NO_DATA, AutonomousRoutine.MIDDLE_RIGHT_SWITCH_MULTI).setMessage(SCALE_NO_DATA_MESSAGE)
        )),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE)
    )),
    RIGHT_HOOK_SWITCH(new DecisionNode(
        new DecisionNode(SWITCH_OWNED_RIGHT, AutonomousRoutine.RIGHT_HOOK),
        new DecisionNode(SWITCH_OWNED_LEFT, AutonomousRoutine.GO_STRAIGHT),
        new DecisionNode(SWITCH_NO_DATA, AutonomousRoutine.GO_STRAIGHT).setMessage(SWITCH_NO_DATA_MESSAGE))),
    CROSS_LINE(new DecisionNode(AutonomousRoutine.GO_STRAIGHT), true),
    STUPID(new DecisionNode(AutonomousRoutine.DRIVE_TIMED));


    private final String name;
    private final DecisionNode root;
    private final boolean isDefault;

    AutoMode(String name, DecisionNode root) {
      this(name, root, false);
    }

    AutoMode(String name, DecisionNode root, boolean isDefault) {
      this.name = name;
      this.root = root;
      this.isDefault = isDefault;
    }

    AutoMode(DecisionNode root, boolean isDefault) {
      StringBuilder nameMaker = new StringBuilder();
      for (String word : this.name().split("_")) {
        nameMaker.append(word.substring(0, 1).toUpperCase()).append(word.substring(1).toLowerCase()).append(" ");
      }
      name = nameMaker.toString().trim();
      this.root = root;
      this.isDefault = isDefault;
    }


    AutoMode(DecisionNode root) {
      StringBuilder nameMaker = new StringBuilder();
      for (String word : this.name().split("_")) {
        nameMaker.append(word.substring(0, 1).toUpperCase()).append(word.substring(1).toLowerCase()).append(" ");
      }
      name = nameMaker.toString().trim();
      this.root = root;
      this.isDefault = false;
    }

  }

  private enum AutonomousRoutine {
    MIDDLE_LEFT_SWITCH_MULTI(() -> new ProfileMultiSwitchAuto("middle_to_left_switch", "left_switch_to_pickup", "pickup_to_left_switch")),
    MIDDLE_RIGHT_SWITCH_MULTI(() -> new ProfileMultiSwitchAuto("middle_to_right_switch", "right_switch_to_pickup", "pickup_to_right_switch")),
    MIDDLE_LEFT_SWITCH_SCALE(() -> new ProfileSwitchScaleAuto("middle_to_left_switch", "left_switch_to_pickup", "pickup_to_left_scale")),
    MIDDLE_RIGHT_SWITCH_SCALE(() -> new ProfileSwitchScaleAuto("middle_to_right_switch", "right_switch_to_pickup", "pickup_to_right_scale")),
    GO_STRAIGHT(() -> new SimpleProfileAuto("go_straight")),
    LEFT_HOOK(() -> new SingleCubeSwitchAuto("left_hook")),
    LEFT_SCALE(() -> new ProfileScaleAuto("left_scale")),
    // LEFT_SCALE_NO_SWITCH(new ProfileDoubleScaleAuto("left_scale", "left_scale_back_to_switch", "left_switch_back_to_scale")),
    // LEFT_DOUBLE_SCALE_THEN_SWITCH(new ProfileDoubleScaleAuto("left_scale_straight", "left_scale_straight_back_to_switch", "left_switch_straight_back_to_scale")),
    // LEFT_SCALE_STRAIGHT(new ProfileScaleAuto("left_scale_straight")),
    MIDDLE_LEFT_SWITCH_1CUBE(() -> new SingleCubeSwitchAuto("middle_to_left_switch")),
    MIDDLE_RIGHT_SWITCH_1CUBE(() -> new SingleCubeSwitchAuto("middle_to_right_switch")),
    SWITCH_DOUBLE_CUBE_LEFT(() -> new SwitchDoubleCube("left")),
    SWITCH_DOUBLE_CUBE_RIGHT(() -> new SwitchDoubleCube("right")),
    RIGHT_HOOK(() -> new SingleCubeSwitchAuto("right_hook")),
    // RIGHT_SCALE(new ProfileScaleAuto("right_scale")),
    DRIVE_TIMED(() -> new DriveTimed(ControlMode.PercentOutput, Tuning.stupidDriveTime, Tuning.stupidDrivePercent));

    private final Supplier<Command> autoCommand;
    private final String defaultMessage;

    AutonomousRoutine(Supplier<Command> autoCommand) {
      this.autoCommand = autoCommand;
      this.defaultMessage = "Running routine " + this.name();
    }

    AutonomousRoutine(Supplier<Command> autoCommand, String defaultMessage) {
      this.autoCommand = autoCommand;
      this.defaultMessage = defaultMessage;
    }

  }

  /**
   * Class for use in a tree structure for deciding what autonomous routine to execute. Each
   * decision node has a {@link #condition} that determines if it's possible to continue pathing
   * down that node. Each decision node either has children, or if it is a leaf, an
   * AutonomousRoutine.
   */
  private static class DecisionNode {

    @NotNull
    private final BooleanSupplier condition;
    @Nullable
    private Message message = null;
    @Nullable
    private final Robot.AutonomousRoutine profile;
    @Nullable
    private final DecisionNode[] children;

    public DecisionNode(@NotNull BooleanSupplier condition, @NotNull Robot.AutonomousRoutine profile) {
      this.condition = condition;
      this.profile = profile;
      this.children = null;
      message = new Message(profile.defaultMessage);
    }

    public DecisionNode(@NotNull Robot.AutonomousRoutine profile) {
      this.condition = () -> (true);
      this.children = null;
      this.profile = profile;
    }

    public DecisionNode(@NotNull BooleanSupplier condition, @NotNull DecisionNode... children) {
      this.condition = condition;
      this.children = children;
      this.profile = null;
    }

    public DecisionNode(@NotNull DecisionNode... children) {
      this.condition = () -> (true);
      this.children = children;
      this.profile = null;
    }

    public DecisionNode setMessage(Message message) {
      this.message = message;
      return this;
    }

    @NotNull
    public BooleanSupplier getCondition() {
      return condition;
    }

    public Optional<Message> getMessage() {
      return Optional.ofNullable(message);
    }

    public Optional<AutonomousRoutine> getProfile() {
      return Optional.ofNullable(profile);
    }

    public Optional<DecisionNode[]> getChildren() {
      return Optional.ofNullable(children);
    }
  }

  private static class Message {
    private String message;
    private boolean isError = false;

    public Message(String message) {
      this.message = message;
    }

    public Message(String message, boolean isError) {
      this.message = message;
      this.isError = isError;
    }

    public void displayMessage() {
      if (isError) {
        System.err.println(message);
      } else {
        System.out.println(message);
      }
    }
  }
}

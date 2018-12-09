package org.team1540.robot2018.testing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2018.Tuning;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.adjustables.AdjustableManager;
import org.team1540.rooster.adjustables.Telemetry;
import org.team1540.rooster.adjustables.Tunable;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.wrappers.ChickenController;
import org.team1540.rooster.wrappers.ChickenTalon;

public class ElevatorTuningRobot extends IterativeRobot {
  @Tunable("Ramp")
  public double clr = 0;
  @Tunable("Peak out fwd")
  public double pof = 1;
  @Tunable("Peak out reverse")
  public double por = -1;

  @Tunable("I Zone")
  public int iZone = 0;

  private SendableChooser<TuningMode> chooser = new SendableChooser<>();
  @Tunable("P")
  public double p = Tuning.elevatorP;
  @Tunable("I")
  public double i = Tuning.elevatorI;
  @Tunable("D")
  public double d = Tuning.elevatorD;
  @Tunable("F")
  public double fUpper = Tuning.elevatorFGoingUp;
  @Tunable("F Down")
  public double fDown = Tuning.elevatorFGoingDown;
  @Tunable("PID Mode")
  public boolean usingPid;
  @Tunable("PID Target")
  public double setpoint;

  @Tunable("Invert Motor 1")
  public boolean invert1 = false;
  @Tunable("Invert Motor 2")
  public boolean invert2 = false;
  private ChickenTalon motor1 = new ChickenTalon(14);
  private ChickenController motor2 = new ChickenTalon(13);

  @Tunable("Invert Sensor")
  public boolean invertSensor;

  @Tunable("Joystick Multiplier")
  public double joystickMultiplier = 0;

  @Tunable("Motion Magic Max Acceleration")
  public int motionMaxAccel = 0;
  @Tunable("Motion Magic Cruise Vel")
  public int motionMaxVel = 0;

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    motor1.setBrake(true);
    motor1.setInverted(invert1);
    motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motor1.configClosedloopRamp(clr);
    motor1.configPeakOutputForward(pof);
    motor1.configPeakOutputReverse(por);
    motor1.config_kP(0, p);
    motor1.config_kI(0, i);
    motor1.config_kD(0, d);
    motor1.config_kF(0, motor1.getSelectedSensorVelocity() > 0 ? fDown : fUpper);
    motor1.setSensorPhase(invertSensor);
    motor1.configMotionAcceleration(motionMaxAccel);
    motor1.configMotionCruiseVelocity(motionMaxVel);

    motor1.config_IntegralZone(0, iZone);

    motor1.configClosedloopRamp(clr);
    motor1.configPeakOutputForward(1);
    motor1.configPeakOutputReverse(-1);

    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor2.setInverted(invert2);
    SmartDashboard.putNumber("Throttle", motor1.getMotorOutputPercent());
    SmartDashboard.putNumber("Current", motor1.getOutputCurrent());
    SmartDashboard.putNumber("Current 2", motor2.getOutputCurrent());
    SmartDashboard.putNumber("Position", motor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Velocity", motor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Trajectory Position", motor1.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Trajectory Velocity", motor1.getActiveTrajectoryVelocity());
    SmartDashboard.putNumber("Error", motor1.getClosedLoopError());

    // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("Camera", 0);
    // camera.setResolution(640, 480);
    // MjpegServer mjpegServer = new MjpegServer("Camera Server", 1182);
    // mjpegServer.setSource(camera);
  }

  @Telemetry("Joystick Position")
  public double joystickPosition = 0;

  @Override
  public void robotInit() {
    // zeroing command
    AdjustableManager.getInstance().add(this);
    joystickPosition = motor1.getSelectedSensorPosition();


    chooser.addDefault("Joystick", TuningMode.JOYSTICK);
    chooser.addObject("PID With Setpoint", TuningMode.PID);
    chooser.addObject("PID With Joystick", TuningMode.PID_JOYSTICK);
    chooser.addObject("PID Move to Position", TuningMode.PID_MTP);
    chooser.addObject("Motion Profiling", TuningMode.MOT_MAGIC);

    SmartDashboard.putData("Chooser", chooser);
    SmartDashboard.putData("Zero Position", new SimpleCommand("Zero position", () -> motor1.setSelectedSensorPosition(0)));
    SmartDashboard.putData("Scheduler", Scheduler.getInstance());
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void teleopInit() {
    joystickPosition = motor1.getSelectedSensorPosition();
  }

  @Override
  public void testInit() {

  }

  private enum TuningMode {JOYSTICK, PID, PID_JOYSTICK, PID_MTP, MOT_MAGIC}

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    switch (chooser.getSelected()) {
      case JOYSTICK:
        motor1.set(ControlMode.PercentOutput, joystick.getRawAxis(1));
        joystickPosition = motor1.getSelectedSensorPosition();
        break;
      case PID:
        motor1.set(ControlMode.Position, setpoint);
        joystickPosition = motor1.getSelectedSensorPosition();
        break;
      case PID_JOYSTICK:
        joystickPosition +=
            Utilities.processDeadzone(joystick.getRawAxis(1), 0.1) * joystickMultiplier;
        motor1.set(ControlMode.Position, joystickPosition);
        break;
      case PID_MTP:
        if (Math.abs(setpoint - joystickPosition) > Math.abs(setpoint - joystickMultiplier)) {
          joystickPosition -= Math.copySign(joystickMultiplier,
              motor1.getSelectedSensorPosition() - setpoint);
        } else {
          joystickPosition = setpoint;
        }
        motor1.set(ControlMode.Position, joystickPosition);
        break;
      case MOT_MAGIC:
        motor1.set(ControlMode.MotionMagic, setpoint);
    }
  }

  @Override
  public void testPeriodic() {

  }
}

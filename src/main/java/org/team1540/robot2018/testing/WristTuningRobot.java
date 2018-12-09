package org.team1540.robot2018.testing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.rooster.adjustables.AdjustableManager;
import org.team1540.rooster.adjustables.Tunable;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.wrappers.ChickenTalon;

public class WristTuningRobot extends IterativeRobot {
  @Tunable("Calibration timeout")
  public double wristCalTimeout = 1;
  @Tunable("Stall current")
  public double stallCurrent = 30;

  @Tunable("I Zone")
  public int iZone = 0;

  private SendableChooser<TuningMode> chooser = new SendableChooser<>();
  @Tunable("P")
  public double p;
  @Tunable("I")
  public double i;
  @Tunable("D")
  public double d;
  @Tunable("F")
  public double f;
  @Tunable("PID Target")
  public double setpoint;

  @Tunable("Invert Motor")
  public boolean invert1 = false;

  private ChickenTalon motor1 = new ChickenTalon(7);

  @Tunable("Invert Sensor")
  public boolean invertSensor;

  @Tunable("Motion Magic Max Acceleration")
  public int motionMaxAccel = 0;
  @Tunable("Motion Magic Cruise Vel")
  public int motionMaxVel = 0;

  private Joystick joystick = new Joystick(0);

  private Command calCmd = new CalibrateWrist();

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    motor1.setBrake(true);
    motor1.setInverted(invert1);
    motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motor1.config_kP(0, p);
    motor1.config_kI(0, i);
    motor1.config_kD(0, d);
    motor1.config_kF(0, f);
    motor1.setSensorPhase(invertSensor);
    motor1.configMotionAcceleration(motionMaxAccel);
    motor1.configMotionCruiseVelocity(motionMaxVel);

    motor1.config_IntegralZone(0, iZone);

    motor1.configClosedloopRamp(0);
    motor1.configPeakOutputForward(1);
    motor1.configPeakOutputReverse(-1);

    SmartDashboard.putNumber("Throttle", motor1.getMotorOutputPercent());
    SmartDashboard.putNumber("Current", motor1.getOutputCurrent());
    SmartDashboard.putNumber("Position", motor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Velocity", motor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Trajectory Position", motor1.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Trajectory Velocity", motor1.getActiveTrajectoryVelocity());
    SmartDashboard.putNumber("Error", motor1.getClosedLoopError());
  }

  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(this);

    chooser.addDefault("Joystick", TuningMode.JOYSTICK);
    chooser.addObject("Calibration", TuningMode.CALIBRATE);
    chooser.addObject("Motion Profiling", TuningMode.MOT_MAGIC);

    SmartDashboard.putData("Mode", chooser);

    // zeroing command
    SimpleCommand zeroCommand = new SimpleCommand("Zero position", () -> motor1.setSelectedSensorPosition(0));
    zeroCommand.setRunWhenDisabled(true);
    SmartDashboard.putData("Zero Position", zeroCommand);
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
    if (chooser.getSelected() == TuningMode.CALIBRATE) {
      calCmd = new CalibrateWrist();
      calCmd.start();
    }
  }

  @Override
  public void testInit() {

  }

  private enum TuningMode {JOYSTICK, CALIBRATE, MOT_MAGIC}

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
        calCmd.cancel();
        motor1.set(ControlMode.PercentOutput, joystick.getRawAxis(1));
        break;
      case MOT_MAGIC:
        calCmd.cancel();
        motor1.set(ControlMode.MotionMagic, setpoint);
    }
  }

  @Override
  public void testPeriodic() {

  }

  public class CalibrateWrist extends Command {

    public CalibrateWrist() {
      super(wristCalTimeout);
    }

    @Override
    protected void initialize() {
      System.out.println("Calibrating wrist...");
      motor1.set(ControlMode.PercentOutput, 1);
    }

    @Override
    protected void end() {
      System.out.println(
          "Wrist calibrated. Position before calibration: " + motor1.getSelectedSensorPosition());
      motor1.setSelectedSensorPosition(0);
      motor1.set(0);
    }

    @Override
    protected boolean isFinished() {
      return motor1.getOutputCurrent() > stallCurrent && isTimedOut();
    }
  }
}

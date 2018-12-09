package org.team1540.robot2018.testing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.adjustables.AdjustableManager;
import org.team1540.rooster.adjustables.Tunable;
import org.team1540.rooster.wrappers.ChickenController;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.wrappers.ChickenVictor;

public class RobotMotorTest extends IterativeRobot {

  private Joystick driver = new Joystick(0);
  private ChickenController[] motors = new ChickenController[]{
      // Drive Left
      new ChickenTalon(1),
      new ChickenTalon(2),
      new ChickenTalon(3),

      // Drive Right
      new ChickenTalon(4),
      new ChickenTalon(5),
      new ChickenTalon(6),

      // Wrist
      new ChickenTalon(7),

      // Climber Tape
      new ChickenVictor(8),

      // Climber Winch
      new ChickenTalon(9),
      new ChickenVictor(10),
      new ChickenVictor(11),
      new ChickenVictor(12),

      // Lift
      new ChickenTalon(13),
      new ChickenTalon(14),

      // Intake
      new ChickenVictor(15),
      new ChickenVictor(16),
      new ChickenTalon(17),
      new ChickenTalon(18),

  };

  private Spark leds = new Spark(9);
  private SendableChooser<Integer>[] motorChoosers;
  private SendableChooser<Integer>[] joystickChoosers;

  @Tunable("[MotorTest] numChoosers (Restart neccecary)")
  public int numChoosers = 1;

  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(this);
    LiveWindow.disableAllTelemetry();
    leds.set(0.35);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void teleopInit() {
    motorChoosers = new SendableChooser[numChoosers];
    joystickChoosers = new SendableChooser[numChoosers];

    for (int motorIndex = 0; motorIndex < numChoosers; motorIndex++) {
      motorChoosers[motorIndex] = new SendableChooser<>();

      motorChoosers[motorIndex].addDefault("(None)", -1);
      for (int cc = 0; cc < motors.length; cc++) {
        motorChoosers[motorIndex].addObject(
            "Motor " + Integer.toString(cc + 1) + " (Forwards)", cc);
      }
      for (int cc = motors.length; cc < motors.length * 2; cc++) {
        motorChoosers[motorIndex].addObject(
            "Motor " + Integer.toString(cc - motors.length + 1) + " (Reversed)", cc);
      }
      SmartDashboard.putData(
          "[MotorTest] MotorChooser " + Integer.toString(motorIndex), motorChoosers[motorIndex]);

      joystickChoosers[motorIndex] = new SendableChooser<>();
      joystickChoosers[motorIndex].addDefault("LeftJoyY", 1);
      joystickChoosers[motorIndex].addObject("LeftJoyX", 0);
      joystickChoosers[motorIndex].addObject("RightJoyY", 5);
      joystickChoosers[motorIndex].addObject("RightJoyX", 4);
      joystickChoosers[motorIndex].addObject("RightTrig", 3);
      joystickChoosers[motorIndex].addObject("LeftTrig", 2);

      SmartDashboard.putData(
          "[MotorTest] JoyChooser " + Integer.toString(motorIndex), joystickChoosers[motorIndex]);
    }

    for (ChickenController motor : motors) {
      if (motor != null) {
        motor.setBrake(true);
        motor.configPeakOutputForward(1);
        motor.configPeakOutputReverse(-1);
        motor.configClosedloopRamp(0);
        motor.configOpenloopRamp(0);
      }
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
    for (int chooserIndex = 0; chooserIndex < motorChoosers.length; chooserIndex++) {
      int selectedIndex = motorChoosers[chooserIndex].getSelected();
      if (selectedIndex != -1) {
        int motorIndex =
            selectedIndex < motors.length ? selectedIndex : (selectedIndex) % motors.length;
        double deadzoneAxis = Utilities.processDeadzone(
            driver.getRawAxis(joystickChoosers[chooserIndex].getSelected()
            ), 0.1);
        int negation = selectedIndex < motors.length ? 1 : -1;
        motors[motorIndex].set(ControlMode.PercentOutput, negation * deadzoneAxis);
      }
    }
  }
}

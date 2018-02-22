package org.team1540.robot2018;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.base.adjustables.AdjustableManager;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.commands.climber.AlignClimber;
import org.team1540.robot2018.commands.climber.WinchOut;
import org.team1540.robot2018.commands.elevator.JoystickElevator;
import org.team1540.robot2018.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2018.commands.groups.FrontScale;
import org.team1540.robot2018.commands.groups.GroundPosition;
import org.team1540.robot2018.commands.groups.IntakeSequence;
import org.team1540.robot2018.commands.intake.EjectCube;
import org.team1540.robot2018.commands.wrist.JoystickWrist;
import org.team1540.robot2018.subsystems.ClimberTapeMeasure;
import org.team1540.robot2018.subsystems.ClimberTurret;
import org.team1540.robot2018.subsystems.ClimberWinch;
import org.team1540.robot2018.subsystems.DriveTrain;
import org.team1540.robot2018.subsystems.Elevator;
import org.team1540.robot2018.subsystems.Intake;
import org.team1540.robot2018.subsystems.Wrist;

public class Robot extends IterativeRobot {
  public static final DriveTrain drivetrain = new DriveTrain();
  public static final Intake intake = new Intake();
  public static final Elevator elevator = new Elevator();
  public static final Wrist wrist = new Wrist();
  public static final ClimberTurret turret = new ClimberTurret();
  public static final ClimberTapeMeasure tape = new ClimberTapeMeasure();
  public static final ClimberWinch winch = new ClimberWinch();

  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(new Tuning());

    Command intakeCommand = new IntakeSequence();
    OI.copilotLB.whenPressed(intakeCommand);
    OI.copilotRB.whenPressed(new EjectCube());
    OI.copilotStart.whenPressed(new SimpleCommand("Stop intake", intake::stop, intake));
    OI.copilotBack.whileHeld(new WinchOut());

    OI.copilotA.whenPressed(new MoveElevatorToPosition(Tuning.elevatorExchangePosition));

    OI.copilotB.whileHeld(new SimpleCommand("Tape out", () -> tape.set(Tuning.tapeOutSpeed), tape));
    OI.copilotY.whileHeld(new SimpleCommand("Tape in", () -> tape.set(Tuning.tapeInSpeed), tape));

    OI.copilotDPadRight.whenPressed(new MoveElevatorToPosition(Tuning.elevatorFrontSwitchPosition));
    OI.copilotDPadLeft.whenPressed(new MoveElevatorToPosition(Tuning.elevatorScalePosition));
    OI.copilotDPadUp.whenPressed(new FrontScale());
    OI.copilotDPadDown.whenPressed(new GroundPosition());

    OI.elevatorJoystickActivation.whileHeld(new JoystickElevator());

    OI.wristJoystickActivation.whileHeld(new ConditionalCommand(new AlignClimber(), new JoystickWrist()) {
      @Override
      protected boolean condition() {
        return OI.copilotLeftTrigger.get();
      }
    });

    OI.copilotRightTriggerSmallPress.whileHeld(new SimpleCommand("Winch In Low", () -> {
      // TODO: Also run tape
      winch.set(Tuning.climberInLowSpeed * Tuning.winchMultiplier);
      // TODO: Is winch out necessary with new design
    }, winch));

    OI.copilotRightTriggerLargePress.whileHeld(new SimpleCommand("Winch In High", () -> {
      // TODO: Also run tape
      winch.set(Tuning.climberInHighSpeed * Tuning.winchMultiplier);
      // TODO: Is winch out necessary with new design
    }, winch));
  }

  @Override
  public void disabledInit() {
    turret.disableServos();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void teleopInit() {
    turret.enableServos();
    turret.init();
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
  }
}

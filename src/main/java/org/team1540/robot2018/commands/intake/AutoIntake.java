package org.team1540.robot2018.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;


public class AutoIntake extends Command {

  public AutoIntake() {
    super(Tuning.intakeMaxTime);
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    setTimeout(Tuning.intakeMaxTime);
    Robot.intake.set(Tuning.IntakeSpeedA, Tuning.IntakeSpeedB);
  }

  @Override
  protected boolean isFinished() {
    return ((Robot.intake.getCurrent() >= Tuning.intakeSpikeCurrent)
        && this.timeSinceInitialized() > Tuning.intakeMinTime)
        || isTimedOut();
  }

  @Override
  protected void end() {
    if (!isTimedOut()) {
      Robot.intake.set(Tuning.intakeHoldSpeed);
    } else {
      Robot.intake.stop();
    }
  }

  @Override
  protected void interrupted() {
    Robot.intake.stop();
  }
}

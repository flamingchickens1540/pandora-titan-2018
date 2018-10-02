package org.team1540.robot2018.commands.arms;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;

public class OpenArms extends Command {
  public OpenArms() {
    requires(Robot.arms);
  }

  @Override
  protected void execute() {
    Robot.arms.set(Tuning.armJoystickConstant, Tuning.armJoystickConstant);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}

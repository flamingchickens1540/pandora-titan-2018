package org.team1540.robot2018.commands.arms;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2018.OI;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;

public class JoystickArms extends Command {

  public JoystickArms() {
    requires(Robot.arms);
  }

  @Override
  protected void execute() {
    // TODO: Invert motors instead of negating set values
    Robot.arms.set(OI.getArmLeftAxis() * Tuning.armJoystickConstant,
        OI.getArmRightAxis() * Tuning.armJoystickConstant);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}

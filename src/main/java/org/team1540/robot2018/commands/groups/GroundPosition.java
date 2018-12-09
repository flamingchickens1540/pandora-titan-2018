package org.team1540.robot2018.commands.groups;

import static org.team1540.robot2018.commands.wrist.CalibrateWrist.CalibratePosition.OUT;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.elevator.CalibrateElevator;
import org.team1540.robot2018.commands.elevator.MoveElevator;
import org.team1540.robot2018.commands.wrist.CalibrateWristMP;
import org.team1540.rooster.util.SimpleConditionalCommand;

public class GroundPosition extends CommandGroup {
  public GroundPosition() {
    addSequential(new CalibrateWristMP(OUT));
    addSequential(new SimpleConditionalCommand(
        () -> Robot.elevator.getPosition() > Tuning.elevatorGroundPosition,
        new MoveElevator(true, Tuning.elevatorGroundPosition)));
    addSequential(new CalibrateElevator());
    addSequential(new InstantCommand());
  }
}

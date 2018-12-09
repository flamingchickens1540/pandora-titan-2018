package org.team1540.robot2018.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.wrist.MoveWrist;
import org.team1540.rooster.util.SimpleConditionalCommand;

public class MoveElevatorSafe extends CommandGroup {

  public MoveElevatorSafe(boolean limitCurrent, double target) {
    addSequential(new SimpleConditionalCommand(
        () -> ((target > Tuning.elevatorObstaclePosition
            && target < Tuning.elevatorObstacleUpperPosition)
            || ((target > Tuning.elevatorObstacleUpperPosition
            && Robot.elevator.getPosition() < Tuning.elevatorObstacleUpperPosition)
            || (target < Tuning.elevatorObstaclePosition
            && Robot.elevator.getPosition() > Tuning.elevatorObstaclePosition)))
            && Robot.wrist.getPosition() < Tuning.wristTransitPosition,
        new MoveWrist(Tuning.wristTransitPosition)));
    addSequential(new MoveElevator(limitCurrent, target));
  }
}

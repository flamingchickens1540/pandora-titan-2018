package org.team1540.robot2018.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.elevator.MoveElevator;
import org.team1540.robot2018.commands.wrist.MoveWristToPosition;

public class Exchange extends CommandGroup {
  public Exchange() {
    addSequential(new MoveWristToPosition(Tuning.wristTransitPosition));
    addSequential(new MoveElevator(Tuning.elevatorExchangePosition));
    addSequential(new MoveWristToPosition(Tuning.wristOutPosition));
  }
}
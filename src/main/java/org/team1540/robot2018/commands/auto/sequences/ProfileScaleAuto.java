package org.team1540.robot2018.commands.auto.sequences;

import static org.team1540.robot2018.commands.wrist.CalibrateWrist.CalibratePosition.OUT;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.elevator.MoveElevator;
import org.team1540.robot2018.commands.groups.GroundPosition;
import org.team1540.robot2018.commands.intake.Eject;
import org.team1540.robot2018.commands.wrist.CalibrateWristMP;
import org.team1540.robot2018.commands.wrist.MoveWrist;
import org.team1540.robot2018.motion.FollowProfile;

public class ProfileScaleAuto extends CommandGroup {
  public ProfileScaleAuto(String name) {
    // this will wait until both the drive commands AND the superstructure are in position before we eject
    addSequential(new CommandGroup() {
      {
        // start driving
        addParallel(new FollowProfile(name));
        // wait a little bit before moving the wrist so we don't hit the wall
        addSequential(new TimedCommand(Tuning.autoElevatorRaiseWait));
        addSequential(new CalibrateWristMP(OUT));
        addParallel(new MoveWrist(Tuning.wristTransitPosition));
        addSequential(new MoveElevator(false, Tuning.elevatorMaxPosition));
        addSequential(new MoveWrist(Tuning.wristBackPosition));
        addSequential(new TimedCommand(0.5));
      }
    });
    addSequential(new Eject(Tuning.intakeEjectSpeedAuto, 1.5));
    addSequential(new GroundPosition());
  }
}

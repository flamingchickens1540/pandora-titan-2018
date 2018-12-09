package org.team1540.robot2018.commands.auto.sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.arms.DropCube;
import org.team1540.robot2018.commands.elevator.MoveElevator;
import org.team1540.robot2018.commands.groups.GroundPosition;
import org.team1540.robot2018.commands.intake.AutoIntake;
import org.team1540.robot2018.commands.intake.Eject;
import org.team1540.robot2018.commands.wrist.MoveWrist;
import org.team1540.robot2018.motion.FollowProfile;
import org.team1540.rooster.util.SimpleCommand;

public class ProfileDoubleScaleAuto extends CommandGroup {
  public ProfileDoubleScaleAuto(String firstApproachProfile, String backupProfile, String secondApproachProfile) {
    // drop the first cube
    addSequential(new ProfileScaleAuto(firstApproachProfile));

    // we end in ground position

    // run the backup profile and pick up the cube
    // this causes the command group to execute these commands at the same time but wait for both to finish before moving on
    addSequential(new CommandGroup() {
      {
        addParallel(new AutoIntake(), 4);
        addParallel(new FollowProfile(backupProfile));
        addSequential(new DropCube(2));
        addParallel(new SimpleCommand("Arm Hold", () -> Robot.arms.set(Tuning.armHoldSpeed), Robot.arms));
      }
    });
    addParallel(new SimpleCommand("Stop opening arms", () -> {}, Robot.arms));

    // go back to the scale, raise elevator, eject cube
    addSequential(new CommandGroup() {
      {
        addParallel(new FollowProfile(secondApproachProfile));
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

package org.team1540.robot2018.commands.auto.sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.arms.DropCube;
import org.team1540.robot2018.commands.elevator.MoveElevator;
import org.team1540.robot2018.commands.groups.GroundPosition;
import org.team1540.robot2018.commands.intake.AutoIntake;
import org.team1540.robot2018.commands.intake.Eject;
import org.team1540.robot2018.commands.wrist.MoveWrist;
import org.team1540.robot2018.motion.FollowProfile;

public class ProfileSwitchScaleAuto extends CommandGroup {
  public ProfileSwitchScaleAuto(String approachProfile, String pickupProfile, String scaleApproachProfile) {
    addSequential(new SingleCubeSwitchAuto(approachProfile));
    addSequential(new CommandGroup() {
      {
        // Intake the cube while driving forwards
        addParallel(new AutoIntake(), 4);
        addParallel(new FollowProfile(pickupProfile));
        addSequential(new DropCube(2));
        addParallel(new SimpleCommand("Arm Hold", () -> Robot.arms.set(Tuning.armHoldSpeed), Robot.arms));
      }
    });

    addSequential(new CommandGroup() {
      {
        // start driving
        addParallel(new FollowProfile(scaleApproachProfile));
        addParallel(new MoveWrist(Tuning.wristTransitPosition));
        addSequential(new TimedCommand(Tuning.autoElevatorRaiseWait));
        addSequential(new MoveElevator(false, Tuning.elevatorMaxPosition));
        addSequential(new MoveWrist(Tuning.wristBackPosition));
        addSequential(new TimedCommand(0.5));
      }
    });
    addSequential(new Eject(Tuning.intakeEjectSpeedAuto, 1.5));
    addSequential(new GroundPosition());
  }
}

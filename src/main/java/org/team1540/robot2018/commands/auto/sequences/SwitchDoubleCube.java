package org.team1540.robot2018.commands.auto.sequences;

import static org.team1540.robot2018.commands.wrist.CalibrateWrist.CalibratePosition.OUT;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.arms.DropCube;
import org.team1540.robot2018.commands.elevator.MoveElevator;
import org.team1540.robot2018.commands.intake.AutoIntake;
import org.team1540.robot2018.commands.intake.Eject;
import org.team1540.robot2018.commands.wrist.CalibrateWristMP;
import org.team1540.robot2018.motion.FollowProfile;
import org.team1540.rooster.util.SimpleCommand;

public class SwitchDoubleCube extends CommandGroup {
  public SwitchDoubleCube(String side) {


    addSequential(new CommandGroup() {
      {
        // Lower the wrist
        addParallel(new CalibrateWristMP(OUT));
        addSequential(new TimedCommand(0.2));
        // Go to the switch and raise elevator at the same time
        addParallel(new FollowProfile("middle_to_" + side + "_switch_multi"));
        addSequential(new MoveElevator(false, Tuning.elevatorFrontSwitchPosition));
      }
    });
    // After arrival, eject the cube
    addSequential(new Eject(Tuning.intakeEjectSpeedAuto, 0.5));

    addSequential(new CommandGroup() {
      {
        // Go back to the middle and move elevator down and recalibrate the wrist
        addParallel(new FollowProfile(side + "_switch_to_middle_multi"));
        addSequential(new TimedCommand(0.2));
        addSequential(new MoveElevator(false, Tuning.elevatorGroundPosition));
        addSequential(new CalibrateWristMP(OUT));
      }
    });

    addSequential(new CommandGroup() {
      {
        // Intake the cube while driving forwards
        addParallel(new AutoIntake(), 4);
        addParallel(new FollowProfile("middle_to_cube_" + side));
        addSequential(new DropCube(2));
        addParallel(new SimpleCommand("Arm Hold", () -> Robot.arms.set(Tuning.armHoldSpeed), Robot.arms));
      }
    });
    addParallel(new SimpleCommand("Stop opening arms", () -> {}, Robot.arms));

    // Go back to the middle
    addSequential(new FollowProfile("cube_to_middle_" + side));

    addSequential(new CommandGroup() {
      {
        // Go to the switch and raise the elevator
        addParallel(new FollowProfile("middle_to_" + side + "_switch_multi"));
        addSequential(new MoveElevator(false, Tuning.elevatorFrontSwitchPosition));
      }
    });
    // After arrival, eject the cube
    addSequential(new Eject(Tuning.intakeEjectSpeedAuto, 0.5));

    // Back up a bit and lower the elevator
    addSequential(new FollowProfile("cube_to_middle"));
    addSequential(new MoveElevator(false, Tuning.elevatorGroundPosition));
  }
}

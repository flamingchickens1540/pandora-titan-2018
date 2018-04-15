package org.team1540.robot2018.commands.auto.sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.base.util.SimpleCommand;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.Tuning;
import org.team1540.robot2018.commands.arms.DropCube;
import org.team1540.robot2018.commands.intake.AutoIntake;
import org.team1540.robot2018.commands.intake.Eject;
import org.team1540.robot2018.commands.wrist.CalibrateWrist.CalibratePosition;
import org.team1540.robot2018.commands.wrist.CalibrateWristMP;
import org.team1540.robot2018.commands.wrist.MoveWrist;
import org.team1540.robot2018.motion.FollowProfile;

public class ProfileMultiSwitchAuto extends CommandGroup {
  public ProfileMultiSwitchAuto(String approachProfile, String pickupProfile1, String returnProfile1) {
    addSequential(new SingleCubeSwitchAuto(approachProfile));
    addSequential(new CommandGroup() {
      {
        // Intake the cube while driving forwards
        addParallel(new AutoIntake(), 4);
        addParallel(new FollowProfile(pickupProfile1));
        addSequential(new DropCube(2));
        addParallel(new SimpleCommand("Arm Hold", () -> Robot.arms.set(Tuning.armHoldSpeed), Robot.arms));
      }
    });

    addSequential(new CommandGroup() {
      {
        addParallel(new FollowProfile(returnProfile1));
        addParallel(new MoveWrist(Tuning.wrist45BackPosition));
      }
    });
    addSequential(new TimedCommand(0.5));
    addSequential(new Eject(Tuning.intakeEjectSpeedAuto, 0.5));
    addSequential(new CalibrateWristMP(CalibratePosition.OUT));
  }
}

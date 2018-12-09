package org.team1540.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2018.RobotMap;
import org.team1540.robot2018.Tuning;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.wrappers.ChickenTalon;

public class Arms extends Subsystem {

  private ChickenTalon armMotorLeft = new ChickenTalon(RobotMap.ARM_LEFT);
  private ChickenTalon armMotorRight = new ChickenTalon(RobotMap.ARM_RIGHT);

  public Arms() {
    armMotorLeft.setInverted(true);
    armMotorRight.setInverted(false);
    armMotorLeft.setBrake(false);
    armMotorRight.setBrake(false);
  }

  public void set(double value) {
    set(value, value);
  }

  public void set(double leftValue, double rightValue) {
    armMotorLeft.set(ControlMode.PercentOutput, leftValue);
    armMotorRight.set(ControlMode.PercentOutput, rightValue);
  }

  public double getCurrent() {
    return armMotorLeft.getOutputCurrent() + armMotorRight.getOutputCurrent();
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new SimpleCommand("Arm Hold", () -> {
      armMotorLeft.set(Tuning.armHoldSpeed);
      armMotorRight.set(Tuning.armHoldSpeed);
    }, this));
  }

}

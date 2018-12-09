package org.team1540.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2018.Robot;
import org.team1540.robot2018.RobotMap;
import org.team1540.robot2018.Tuning;
import org.team1540.rooster.wrappers.ChickenVictor;

public class Intake extends Subsystem {
  PowerDistributionPanel pdp = new PowerDistributionPanel();

  private ChickenVictor intakeMotorA = new ChickenVictor(RobotMap.INTAKE_A);
  private ChickenVictor intakeMotorB = new ChickenVictor(RobotMap.INTAKE_B);

  public Intake() {
    intakeMotorA.setInverted(true);
    intakeMotorB.setInverted(false);
  }

  @Override
  protected void initDefaultCommand() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("[Intake] getCurrent", getCurrent());
  }

  public double getCurrent() {
    if (Tuning.isPandora) {
      return pdp.getCurrent(5) - Robot.wrist.getCurrent() - Robot.arms.getCurrent();
    } else {
      return pdp.getCurrent(10) + pdp.getCurrent(11)
          - Robot.wrist.getCurrent() - Robot.arms.getCurrent();
    }
  }

  public void set(double value) {
    set(value, value);
  }

  public void set(double valueA, double valueB) {
    intakeMotorA.set(ControlMode.PercentOutput, valueA);
    intakeMotorB.set(ControlMode.PercentOutput, valueB);
  }

  public void holdCube() {
    set(Tuning.intakeHoldSpeed);
  }
}

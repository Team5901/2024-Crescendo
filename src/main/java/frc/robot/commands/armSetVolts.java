package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class armSetVolts extends Command {

  private Arm arm;
  private double setVolts;

  public armSetVolts(double setVolts, Arm arm) {
    this.setVolts = setVolts;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setVoltage(setVolts);
  }
}

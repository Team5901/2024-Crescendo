package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class armSetVolts extends Command {
  private Intake intake;
  private Arm arm;
  private double setVolts;

  public armSetVolts(double setVolts, Arm arm) {
    this.setVolts = setVolts;
    this.arm = arm;

    addRequirements(arm, intake);
  }

  @Override
  public void initialize() {
    arm.setVoltage(setVolts);
  }
}

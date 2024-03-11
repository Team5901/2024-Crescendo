package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class armSetVelocity extends Command {
  private Intake intake;
  private Arm arm;
  private double setVelocity;

  public armSetVelocity(double setVelocity, Arm arm) {
    this.setVelocity = setVelocity;
    this.arm = arm;

    addRequirements(arm, intake);
  }

  @Override
  public void initialize() {
    arm.setVelocity(setVelocity);
  }
}

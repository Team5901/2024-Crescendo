package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class setIntakeVolts extends Command {

  private Intake intake;
  private double setpointVolts;

  public setIntakeVolts(double setpointVolts, Intake intake) {
    this.setpointVolts = setpointVolts;
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setVoltage(setpointVolts);
  }

  // @Override
  // public boolean isFinished() {
  //   return Math.abs(intake.getVelocityRPM() - setpointVolts)
  //       < Constants.IntakeSubsystem.goalToleranceVelocity;
  // }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class setIntakeRPM extends Command {

  private Intake intake;
  private double setpointRPM;

  public setIntakeRPM(double setpointRPM, Intake intake) {
    this.setpointRPM = setpointRPM;
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.runVelocity(setpointRPM);
  }

  // @Override
  // public boolean isFinished() {
  //   return Math.abs(intake.getVelocityRPM() - setpointRPM)
  //       < Constants.IntakeSubsystem.goalToleranceVelocity;
  // }
}

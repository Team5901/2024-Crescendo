package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoot.Shoot;

public class setShooterVolts extends Command {
  private Intake intake;
  private Shoot shoot;
  private double setpointRPM;

  public setShooterVolts(double setpointRPM, Shoot shoot, Intake intake) {
    this.setpointRPM = setpointRPM;
    this.shoot = shoot;
    this.intake = intake;
    addRequirements(shoot, intake);
  }

  @Override
  public void initialize() {
    shoot.runVelocity(setpointRPM);
    intake.setVoltage(Constants.IntakeSubsystem.intakeShootNoteVoltage);
  }

  // @Override
  // public boolean isFinished() {
  //   boolean a =
  //       Math.abs(shoot.getVelocityRPM() - setpointRPM)
  //           < Constants.ShootSubsystem.goalToleranceVelocity;
  //   boolean b =
  //       Math.abs(intake.getVelocityRPM() - Constants.IntakeSubsystem.intakeShootNoteVelRPM)
  //           < Constants.IntakeSubsystem.goalToleranceVelocity;
  //   return a && b;
  // }
}

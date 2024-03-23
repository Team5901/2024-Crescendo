package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.slider.Slider;

public class AutoShootFarSpeaker extends SequentialCommandGroup {

  public AutoShootFarSpeaker(Slider slider, Arm arm, Shoot shoot, Intake intake) {
    addCommands(
        new goToAimFarSpeaker(arm, slider),
        new setShooterRPM(Constants.ShootSubsystem.shootSpeakerVelRPM, shoot)
            .withTimeout(2)
            .andThen(
                new setIntakeRPM(Constants.IntakeSubsystem.intakeShootNoteVelRPM, intake)
                    .withTimeout(2)),
        new WaitCommand(1),
        new InstantCommand(shoot::stop, shoot)
            .withTimeout(0.25)
            .alongWith(new InstantCommand(intake::stop, intake))
            .withTimeout(0.25),
        new goToIntakeIn(slider, arm).withTimeout(0.25));
  }
}

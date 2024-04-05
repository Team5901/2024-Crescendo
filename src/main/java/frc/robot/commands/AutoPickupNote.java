package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.slider.Slider;
import java.util.function.BooleanSupplier;

public class AutoPickupNote extends SequentialCommandGroup {

  public AutoPickupNote(
      Slider slider, Arm arm, Intake intake, Shoot shoot, DigitalInput intakesensor) {
    addRequirements(arm, slider, intake);
    BooleanSupplier flippedIntake = () -> !intakesensor.get();
    addCommands(
        new goToIntakeOut(slider, arm)
            .andThen(
                new setShooterRPM(Constants.ShootSubsystem.holdNoteVelRPM, shoot)
                    .alongWith(
                        new setIntakeRPMDetectNote(
                                Constants.IntakeSubsystem.intakeInNoteVelRPM,
                                intake,
                                intakesensor,
                                slider,
                                arm)
                            .withTimeout(0.5))));
    // new WaitUntilCommand(flippedIntake),
    // new WaitCommand(1),
    // (new InstantCommand(intake::stop, intake)));
  }
}

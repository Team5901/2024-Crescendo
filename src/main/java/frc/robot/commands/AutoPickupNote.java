package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.slider.Slider;

public class AutoPickupNote extends SequentialCommandGroup {

  public AutoPickupNote(Slider slider, Arm arm, Intake intake, DigitalInput intakesensor) {
    addCommands(
        new goToIntakeOut(slider, arm)
            .withTimeout(1)
            .alongWith(
                new setIntakeRPM(Constants.IntakeSubsystem.intakeInNoteVelRPM, intake)
                    .withTimeout(2)),
        new WaitUntilCommand(intakesensor::get).andThen(new InstantCommand(intake::stop, intake)));
  }
}

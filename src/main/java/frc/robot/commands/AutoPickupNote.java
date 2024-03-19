package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.slider.Slider;

public class AutoPickupNote extends SequentialCommandGroup {

  public AutoPickupNote(Slider slider, Arm arm, Intake intake) {
    addCommands(
        new goToIntakeOut(slider, arm).withTimeout(.5),
        new setIntakeRPM(Constants.IntakeSubsystem.intakeInNoteVelRPM, intake));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.slider.Slider;

public class AutoPickupNoteP2 extends SequentialCommandGroup {

  public AutoPickupNoteP2(Slider slider, Arm arm, Intake intake) {
    addCommands(new InstantCommand(intake::stop, intake), new goToIntakeIn(slider, arm));
  }
}

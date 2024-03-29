package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.slider.Slider;
import java.util.function.BooleanSupplier;

public class AutoPickupNote extends SequentialCommandGroup {

  public AutoPickupNote(
      Slider slider, Arm arm, Intake intake, Shoot shoot, DigitalInput FrontSensor,DigitalInput RearSensor) {
    
    addCommands(
        new goToIntakeOut(slider, arm)
            .alongWith(new TuneNotePosition(intake, FrontSensor, RearSensor)));
  }
}

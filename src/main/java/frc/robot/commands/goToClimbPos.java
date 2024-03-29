package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

public class goToClimbPos extends SequentialCommandGroup {
  public goToClimbPos(Arm arm, Slider slider) {
    addCommands(
        new ArmRotateGoToPosition(100, 1, arm)
            .alongWith(new ArmSliderGoToPosition(6.0, 1, slider))
            .withTimeout(1));
  }
}

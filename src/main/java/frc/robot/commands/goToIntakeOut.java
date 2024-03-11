package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToIntakeOut extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing

  public goToIntakeOut(Slider slider, Arm arm) {
    addCommands(
        new ArmSliderGoToPosition(
                Constants.SliderSubsystem.sliderIntakeOut,
                Constants.SliderSubsystem.goalTolerance,
                slider)
            .withTimeout(1), // Extends the intake arm with a timeout of 1 second
        new ArmRotateGoToPosition(
            Constants.ArmSubsystem.armPosOut,
            Constants.ArmSubsystem.goalTolerance,
            arm) // Rotate's the arm intake
        );
  }
}

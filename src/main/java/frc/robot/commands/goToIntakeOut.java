package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToIntakeOut extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  private double startAngle;
  public goToIntakeOut(Slider slider, Arm arm) {
    startAngle = arm.getAngle();

    if (startAngle <10) { // if reasonably low, send slider, then arm
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
    } else {
      addCommands( // if too high up, bring slider in, then lower arm, then send slider out, then bring arm down again
      new ArmSliderGoToPosition(
                Constants.SliderSubsystem.sliderIntakeIn,
                Constants.SliderSubsystem.goalTolerance,
                slider)
            .alongWith(
              new ArmRotateGoToPosition(
            Constants.ArmSubsystem.armPosIn,
            Constants.ArmSubsystem.goalTolerance,
            arm)),  
       // Rotate's the arm intake
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
}

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
    // If arm above 0 degrees, raise to 5 degrees, extend slider, move arm down
    if (startAngle >= 30) {

      addCommands(
          new ArmRotateGoToPosition(5, Constants.ArmSubsystem.goalTolerance, arm).withTimeout(.5),
          new ArmSliderGoToPosition(
                  Constants.SliderSubsystem.sliderIntakeOut,
                  Constants.SliderSubsystem.goalTolerance,
                  slider)
              .withTimeout(.5), // Extends the intake arm with a timeout of 1 second
          new ArmRotateGoToPosition(
              Constants.ArmSubsystem.armPosOut,
              Constants.ArmSubsystem.goalTolerance,
              arm) // Rotate's the arm intake
          );
      // Else already at intake out and just reaffirm position setpoints
    } else {
      addCommands(
          // new ArmRotateGoToPosition(45, Constants.ArmSubsystem.goalTolerance,
          // arm).withTimeout(.5),
          new ArmSliderGoToPosition(
                  Constants.SliderSubsystem.sliderIntakeOut,
                  Constants.SliderSubsystem.goalTolerance,
                  slider)
              .withTimeout(.25), // Extends the intake arm with a timeout of 1 second
          new ArmRotateGoToPosition(
              Constants.ArmSubsystem.armPosOut, Constants.ArmSubsystem.goalTolerance, arm));
    }
  }
}

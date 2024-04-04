package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToIntakeIn extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  private double startAngle;

  public goToIntakeIn(Slider slider, Arm arm) {
    startAngle = arm.getAngle();
    addRequirements(slider, arm);

    if (startAngle <= 10) {
      addCommands(
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosIn, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(.1), // Intake rotates arm in.
          new ArmSliderGoToPosition(
                  Constants.SliderSubsystem.sliderIntakeIn,
                  Constants.SliderSubsystem.goalTolerance,
                  slider)
              .withTimeout(1) // intake Extend's arm in
          );
    } else {
      addCommands(
          new ArmSliderGoToPosition(
                  Constants.SliderSubsystem.sliderIntakeIn,
                  Constants.SliderSubsystem.goalTolerance,
                  slider)
              .withTimeout(1),
          new ArmRotateGoToPosition(
              Constants.ArmSubsystem.armPosIn, Constants.ArmSubsystem.goalTolerance, arm));
    }
  }
}

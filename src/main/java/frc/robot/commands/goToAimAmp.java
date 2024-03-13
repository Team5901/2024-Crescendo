package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToAimAmp extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm arm;
  double startAngle;
  Slider slider;
  ArmSliderGoToPosition ExtendCommand;
  ArmDashboardRotate RotateCommand;

  public goToAimAmp(Arm arm, Slider slider) {
    double startAngle = arm.getAngle();
    if (startAngle <= 5) {
      addCommands(
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosIn, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(1), // Intake rotates arm in.
          new ArmSliderGoToPosition(
              Constants.SliderSubsystem.sliderIntakeIn,
              Constants.SliderSubsystem.goalTolerance,
              slider), // intake Extend's arm in
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosAmp, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(1), // Intake rotates arm in.
          new ArmSliderGoToPosition(
              Constants.SliderSubsystem.sliderAmp,
              Constants.SliderSubsystem.goalTolerance,
              slider));
    } else {
      addCommands(
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosAmp, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(1), // Intake rotates arm in.
          new ArmSliderGoToPosition(
              Constants.SliderSubsystem.sliderAmp,
              Constants.SliderSubsystem.goalTolerance,
              slider));
    }
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToAimSpeaker extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm arm;
  double startAngle, startExtension;
  Slider slider;
  ArmSliderGoToPosition ExtendCommand;
  ArmDashboardRotate RotateCommand;

  public goToAimSpeaker(Arm arm, Slider slider) {
    double startAngle = arm.getAngle();
    if (startAngle <= 10) {
      addCommands(
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosIn, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(.5), // Intake rotates arm in.
          new ArmSliderGoToPosition(
                  Constants.SliderSubsystem.sliderIntakeIn,
                  Constants.SliderSubsystem.goalTolerance,
                  slider)
              .withTimeout(1)
              .alongWith( // intake Extend's arm in
                  new ArmRotateGoToPosition(
                          Constants.ArmSubsystem.armPosSpeaker,
                          Constants.ArmSubsystem.goalTolerance,
                          arm)
                      .withTimeout(1) // Intake rotates arm in.
                  ));
    } else {
      addCommands(
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosSpeaker, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(1)
              .alongWith( // Intake rotates arm in.
                  new ArmSliderGoToPosition(
                          Constants.SliderSubsystem.sliderSpeaker,
                          Constants.SliderSubsystem.goalTolerance,
                          slider)
                      .withTimeout(1)));
    }
  }
}

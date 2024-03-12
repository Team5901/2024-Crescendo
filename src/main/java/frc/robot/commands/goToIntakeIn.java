package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToIntakeIn extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing

  public goToIntakeIn(Slider slider, Arm arm) { // ASSUMES we start at intake Out
    double startAngle = arm.getAngle();

    if (startAngle <= 5) {
      addCommands(
          new ArmRotateGoToPosition(
                  Constants.ArmSubsystem.armPosIn, Constants.ArmSubsystem.goalTolerance, arm)
              .withTimeout(1), // Intake rotates arm in.
          new ArmSliderGoToPosition(
              Constants.SliderSubsystem.sliderIntakeIn,
              Constants.SliderSubsystem.goalTolerance,
              slider) // intake Extend's arm in
          );
    } else {
      addCommands(
          new ArmSliderGoToPosition(
              Constants.SliderSubsystem.sliderIntakeIn,
              Constants.SliderSubsystem.goalTolerance,
              slider),
          new ArmRotateGoToPosition(
              Constants.ArmSubsystem.armPosIn, Constants.ArmSubsystem.goalTolerance, arm));
    }
  }

  // public void goToAimSpeaker() {}

  // public void goToAimAmp() {
  //   // check if below/at intake out, move to speaker,
  //   if (startAngle < MovementPositions.IntakeOutDeg) {
  //     new InstantCommand(() -> arm.setAngleSetPoint(MovementPositions.AimSpeakerDeg))
  //         .andThen(
  //             new InstantCommand(() -> arm.setAngleSetPoint(MovementPositions.IntakeInDeg))
  //                 .andThen(
  //                     new InstantCommand(
  //                         () -> slider.setPositionSetPoint(MovementPositions.IntakeInDeg))));
  //   }

  //   // check if below/at aim speaker, angle
  //   if (startAngle < MovementPositions.AimSpeakerDeg) {
  //     new InstantCommand(() -> arm.setAngleSetPoint(MovementPositions.IntakeInDeg))
  //         .andThen(
  //             new InstantCommand(
  //                 () ->
  //                     slider.setPositionSetPoint(
  //                         MovementPositions.IntakeInDeg))); // move arm up, THEN in
  //   }

  //   // check if below/at intake in
  //   if (startAngle < MovementPositions.IntakeInDeg) {
  //     new InstantCommand(() -> arm.setAngleSetPoint(MovementPositions.AimAmpDeg));
  //   }
  //   // if at aim amp, do nothing/ reset goalpoint
  //   if (startAngle < MovementPositions.AimAmpDeg) {
  //     new InstantCommand(() -> arm.setAngleSetPoint(MovementPositions.AimAmpDeg));
  //   }
  // }

  public void goToANGLESmartDashboard(Arm arm) {
    double DashboardVal =
        SmartDashboard.getNumber("Arm Angle INPUT", Constants.ArmSubsystem.armPosSpeaker);
    if (DashboardVal < Constants.ArmSubsystem.armSoftLimitLowerAngle) {
      new ArmRotateGoToPosition(
          Constants.ArmSubsystem.armSoftLimitLowerAngle, Constants.ArmSubsystem.goalTolerance, arm);
    } else if (DashboardVal > Constants.ArmSubsystem.armSoftLimitUpperAngle) {
      new ArmRotateGoToPosition(
          Constants.ArmSubsystem.armSoftLimitUpperAngle, Constants.ArmSubsystem.goalTolerance, arm);
    } else {
      new ArmRotateGoToPosition(DashboardVal, Constants.ArmSubsystem.goalTolerance, arm);
    }
  }

  public void goToSLIDERSmartDashboard(Slider slider) {
    double DashboardVal =
        SmartDashboard.getNumber("Slider INPUT", Constants.SliderSubsystem.sliderIntakeOut);
    if (DashboardVal < Constants.SliderSubsystem.sliderSoftLimitLowerInch) {
      new ArmSliderGoToPosition(
          Constants.SliderSubsystem.sliderSoftLimitLowerInch,
          Constants.SliderSubsystem.goalTolerance,
          slider);
    } else if (DashboardVal > Constants.SliderSubsystem.sliderSoftLimitUpperInch) {
      new ArmSliderGoToPosition(
          Constants.SliderSubsystem.sliderSoftLimitUpperInch,
          Constants.SliderSubsystem.goalTolerance,
          slider);
    } else {
      new ArmSliderGoToPosition(DashboardVal, Constants.SliderSubsystem.goalTolerance, slider);
    }
  }
}

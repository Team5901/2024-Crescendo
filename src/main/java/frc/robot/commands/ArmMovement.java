package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class ArmMovement extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm arm;
  double startAngle, startExtension;
  Slider slider;
  ArmExtend ExtendCommand;
  ArmRotate RotateCommand;

  public void goToIntakeOut(Slider slider, Arm arm) { // Assumes we start at intake in ONLY
    // check if above/at aim amp
    new InstantCommand(() -> ExtendCommand.goToIntakeOut(slider))
        .withTimeout(1)
        .andThen(new InstantCommand(() -> RotateCommand.goToIntakeOut(arm)));
    // if (startAngle > MovementPositions.AimAmpDeg) {
    //   new InstantCommand(() -> armsubsystem.setAngleSetPoint(MovementPositions.AimSpeakerDeg));
    // }
    // // check if below/at intake in
    // if (startAngle > MovementPositions.IntakeInDeg ) {
    //   new InstantCommand(()->
    // slidersubsystem.setPositionSetPoint(MovementPositions.IntakeOutInch)).andThen(
    //     new InstantCommand( ()-> armsubsystem.setAngleSetPoint(MovementPositions.IntakeOutDeg)));
    // }
    // check if below/ at aim speaker

    // if at intake out, dont move
  }

  public void goToIntakeIn(Slider slider, Arm arm) { // ASSUMES we start at intake Out
    new InstantCommand(() -> RotateCommand.goToIntakeIn(arm)) // rotate up to our intake in angle
        .withTimeout(1)
        .andThen(new InstantCommand(() -> ExtendCommand.goToIntakeIn(slider))); // then retract
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
}

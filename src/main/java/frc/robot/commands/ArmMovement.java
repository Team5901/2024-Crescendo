package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MovementPositions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java
public class ArmMovement extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm armsubsystem;
  double startAngle, startExtension;
  Slider slidersubsystem;

  public void goToAimAmp() {
    // check if below/at intake out, move to speaker,
    if (startAngle < MovementPositions.IntakeOutDeg) {
      new InstantCommand(() -> armsubsystem.setAngleSetPoint(MovementPositions.AimSpeakerDeg)).andThen(
        new InstantCommand(() -> armsubsystem.setAngleSetPoint(MovementPositions.IntakeInDeg)).andThen(
        new InstantCommand(() -> slidersubsystem.setPositionSetPoint(MovementPositions.IntakeInDeg)))
      );
    }

    // check if below/at aim speaker, angle
    if (startAngle < MovementPositions.AimSpeakerDeg) {
      new InstantCommand(() -> armsubsystem.setAngleSetPoint(MovementPositions.IntakeInDeg)).andThen(
        new InstantCommand(() -> slidersubsystem.setPositionSetPoint(MovementPositions.IntakeInDeg))); // move arm up, THEN in
    }

    // check if below/at intake in
    if (startAngle < MovementPositions.IntakeInDeg) {
      new InstantCommand( () -> armsubsystem.setAngleSetPoint(MovementPositions.AimAmpDeg));
    }
    // if at aim amp, do nothing/ reset goalpoint
    if (startAngle< MovementPositions.AimAmpDeg) {
      new InstantCommand(()->armsubsystem.setAngleSetPoint(MovementPositions.AimAmpDeg));
    }
  }

  public void goToIntakeOut() {
    // check if above/at aim amp
    if (startAngle > MovementPositions.AimAmpDeg) {
      new InstantCommand(() -> armsubsystem.setAngleSetPoint(MovementPositions.AimSpeakerDeg));
    }
    // check if below/at intake in
    if (startAngle > MovementPositions.IntakeInDeg ) {
      new InstantCommand(()-> slidersubsystem.setPositionSetPoint(MovementPositions.IntakeOutInch)).andThen(
        new InstantCommand( ()-> armsubsystem.setAngleSetPoint(MovementPositions.IntakeOutDeg)));
    }
    // check if below/ at aim speaker
    
    // if at intake out, dont move
  }

  public void goToAimSpeaker() {}

  public void goToIntakeIn() {}

 
}

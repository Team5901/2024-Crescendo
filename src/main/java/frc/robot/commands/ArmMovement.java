package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

public class ArmMovement extends Command {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm armsubsystem;
  double startAngle, startExtension;
  Slider slidersubsystem;

  public void ArmMove() {}

  @Override
  public void initialize() {
    startExtension = slidersubsystem.getPosition();
    startAngle = armsubsystem.getAngle();
  }

  @Override
  public boolean isFinished() {

    return true;
  }
}

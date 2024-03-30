package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

public class SmartClimbPos2 extends Command {
  private Arm arm;
  private Slider slider;
  private boolean a, b;
  private double armSetpoint = -10;
  private double sliderHangSetpoint=1;
  public SmartClimbPos2(Arm arm, Slider slider) {
    this.arm = arm;
    this.slider = slider;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, slider);
  }

  @Override
  public void initialize() {
    a = false;
    b = false;
    arm.setConstraints(
                    Constants.ArmSubsystem.maxVelocityDegreesPerSec * 0.5,
                    Constants.ArmSubsystem.maxAccelerationDegreesPerSec * 0.5);
  }

  @Override
  public void execute() {
    slider.setPositionSetPoint(Constants.SliderSubsystem.sliderIntakeIn);
    if (slider.getPosition() < 3){
        arm.setAngleSetPoint(armSetpoint);
    }


  }
  @Override
  public void end(boolean interrupted) {
    arm.setConstraints(
                    Constants.ArmSubsystem.maxVelocityDegreesPerSec,
                    Constants.ArmSubsystem.maxAccelerationDegreesPerSec);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    a =
        Math.abs(slider.getPosition() - Constants.SliderSubsystem.sliderIntakeIn)
            < sliderHangSetpoint;
    b =
        Math.abs(arm.getAngle() - armSetpoint)
            < Constants.ArmSubsystem.goalTolerance;
    return a && b;
  }

}

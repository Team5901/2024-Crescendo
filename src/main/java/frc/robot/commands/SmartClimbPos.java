package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

public class SmartClimbPos extends Command {
  private Arm arm;
  private Slider slider;
  private boolean a, b;
  public SmartClimbPos(Arm arm, Slider slider) {
    this.arm = arm;
    this.slider = slider;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, slider);  
  }

  @Override
  public void initialize() {
    a = false;
    b = false;
  }

  @Override
  public void execute() {
    arm.setAngleSetPoint(100);
    if (arm.getAngle() > -5) {
      slider.setPositionSetPoint(6);
    }
  }

  @Override
  public boolean isFinished() {
    a =
        Math.abs(slider.getPosition() - 6)
            < Constants.SliderSubsystem.goalTolerance;
    b =
        Math.abs(arm.getAngle() - 100)
            < Constants.ArmSubsystem.goalTolerance;
    return a && b;
  }



}

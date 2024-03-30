package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class SmartAimFarSpeaker extends Command {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm arm;
  double startAngle, startExtension;
  Slider slider;
  ArmSliderGoToPosition ExtendCommand;
  ArmDashboardRotate RotateCommand;
  boolean a, b;
  public SmartAimFarSpeaker(Arm arm, Slider slider) {
    this.arm=arm;
    this.slider=slider;
    addRequirements(arm,slider);
  }
  @Override
  public void initialize() {
    a = false;
    b = false;
  }

  @Override
  public void execute() {
    arm.setAngleSetPoint(Constants.ArmSubsystem.armPosFarSpeaker);
    if (arm.getAngle() > -5) {
      slider.setPositionSetPoint(Constants.SliderSubsystem.sliderIntakeIn);
    }
  }

  @Override
  public boolean isFinished() {
    a =
        Math.abs(slider.getPosition() - Constants.SliderSubsystem.sliderIntakeIn)
            < Constants.SliderSubsystem.goalTolerance;
    b =
        Math.abs(arm.getAngle() - Constants.ArmSubsystem.armPosFarSpeaker)
            < Constants.ArmSubsystem.goalTolerance;
    return a && b;
  }

}

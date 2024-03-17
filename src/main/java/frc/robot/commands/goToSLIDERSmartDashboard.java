package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToSLIDERSmartDashboard extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing

  public goToSLIDERSmartDashboard(Slider slider) {
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

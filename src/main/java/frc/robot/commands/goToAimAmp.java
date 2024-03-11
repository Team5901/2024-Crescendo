package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;
// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToAimAmp extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing
  Arm arm;
  double startAngle, startExtension;
  Slider slider;
  ArmSliderGoToPosition ExtendCommand;
  ArmDashboardRotate RotateCommand;

  public goToAimAmp(Arm arm, Slider slider) {}
}

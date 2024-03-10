package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

// https://github.com/Jon-Zimmerman/FRC_Electric_Eels_2023/blob/main/src/main/java/frc/robot/autos/Bottom_Cube_Extended_Cube.java

public class goToANGLESmartDashboard extends SequentialCommandGroup {
  // create method that gracefully extends intake head at low angles to avoid crashing

  public goToANGLESmartDashboard(Arm arm) {
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
}

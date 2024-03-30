// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

public class SmartIntakeOut extends Command {
  private Arm arm;
  private Slider slider;

  /** Creates a new IntakeOutStateMachine. */
  public SmartIntakeOut(Arm arm, Slider slider) {
    this.arm = arm;
    this.slider = slider;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, slider);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getAngle() > 5) {
      arm.setAngleSetPoint(5);
    } else if (slider.getPosition() >= 6) { // gives us our tolerance
      arm.setAngleSetPoint(-5);
    }

    if (arm.getAngle() < 45) {
      slider.setPositionSetPoint(10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return slider.getPosition() > 9 && arm.getAngle() < -4;
  }
}

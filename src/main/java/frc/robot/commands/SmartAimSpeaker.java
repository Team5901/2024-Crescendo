// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.slider.Slider;

public class SmartAimSpeaker extends Command {
  private Arm arm;
  private Slider slider;
  private boolean a,b;
  /** Creates a new IntakeOutStateMachine. */
  public SmartAimSpeaker(Arm arm, Slider slider) {
    this.arm = arm;
    this.slider = slider;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, slider);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    a=false;
    b=false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setAngleSetPoint(Constants.ArmSubsystem.armPosSpeaker);
    if (arm.getAngle() > -5) {
      slider.setPositionSetPoint(Constants.SliderSubsystem.sliderIntakeIn);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    a=Math.abs(slider.getPosition()-Constants.SliderSubsystem.sliderIntakeIn) < Constants.SliderSubsystem.goalTolerance;
    b=Math.abs(arm.getAngle() - Constants.ArmSubsystem.armPosSpeaker ) <Constants.ArmSubsystem.goalTolerance;
    return  a && b;
  }
}

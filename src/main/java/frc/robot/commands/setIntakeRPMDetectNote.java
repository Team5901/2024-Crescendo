package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.slider.Slider;

public class setIntakeRPMDetectNote extends Command {

  private Intake intake;
  private double setpointRPM;
  private DigitalInput IntakeNoteDetector;
  private Slider slider;
  private Arm arm;
  private double flag;

  public setIntakeRPMDetectNote(
      double setpointRPM, Intake intake, DigitalInput IntakeNoteDetector, Slider slider, Arm arm) {
    this.setpointRPM = setpointRPM;
    this.intake = intake;
    this.IntakeNoteDetector = IntakeNoteDetector;
    this.arm = arm;
    this.slider = slider;
    addRequirements(intake, slider, arm);
  }

  @Override
  public void initialize() {
    intake.runVelocity(setpointRPM);
  }

  @Override
  public void execute() {
    if (IntakeNoteDetector.get() == false) {
      intake.stop();
      arm.setAngleSetPoint(Constants.ArmSubsystem.armPosIn);
      if (arm.getAngle() > 2) {
        slider.setPositionSetPoint(Constants.SliderSubsystem.sliderIntakeIn);
        flag = 1;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return flag == 1;
  }
}

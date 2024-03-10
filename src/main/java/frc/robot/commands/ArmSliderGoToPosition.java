package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.slider.Slider;

/*
  https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
  https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html#lambda-expressions-java
  https://docs.wpilib.org/en/stable/docs/software/basic-programming/functions-as-data.html#lambda-expressions-in-java
*/
public class ArmSliderGoToPosition extends Command {

  private Slider m_Slider;
  private double m_setpoint;
  private double goal_tolerance;
  /**
   * Create a new ArmSliderGoToPosition command.
   *
   * @param setpointInch The setpoint to set the slider to
   * @param goalTolerance closeness to end command at
   * @param slider The slider to use
   */
  public ArmSliderGoToPosition(double setpointInch, double goalTolerance, Slider slider) {
    m_Slider = slider;
    m_setpoint = setpointInch;
    goal_tolerance = goalTolerance;
    addRequirements(m_Slider);
  }

  // public void goToIntakeOut(Slider slider) {
  //   m_Slider = slider;
  //   m_setpoint = Constants.SliderSubsystem.sliderIntakeOut;
  //   goal_tolerance = Constants.SliderSubsystem.goalTolerance;
  //   addRequirements(m_Slider);
  // }

  // public void goToIntakeIn(Slider slider) {
  //   m_Slider = slider;
  //   m_setpoint = Constants.SliderSubsystem.sliderIntakeIn;
  //   goal_tolerance = Constants.SliderSubsystem.goalTolerance;
  //   addRequirements(m_Slider);
  // }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_Slider.setPositionSetPoint(m_setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {

    return m_Slider.atSetpoint(goal_tolerance);
  }
}
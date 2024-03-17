package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.slider.Slider;

/*
  https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
  https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html#lambda-expressions-java
  https://docs.wpilib.org/en/stable/docs/software/basic-programming/functions-as-data.html#lambda-expressions-in-java
*/
public class ArmDashboardSlider extends Command {

  private Slider m_Slider;
  private double goal_tolerance;
  /**
   * Create a new ArmSliderGoToPosition command.
   *
   * @param setpointInch The setpoint to set the slider to
   * @param goalTolerance closeness to end command at
   * @param slider The slider to use
   */
  public ArmDashboardSlider(double goalTolerance, Slider slider) {
    m_Slider = slider;
    goal_tolerance = goalTolerance;
    addRequirements(m_Slider);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_Slider.setPositionSetPoint(SmartDashboard.getNumber("Slider INPUT", 0));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {

    return m_Slider.atSetpoint(goal_tolerance);
  }
}

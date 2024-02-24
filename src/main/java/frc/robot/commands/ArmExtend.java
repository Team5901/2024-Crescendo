package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.slider.Slider;

/*
  https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
  https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html#lambda-expressions-java
  https://docs.wpilib.org/en/stable/docs/software/basic-programming/functions-as-data.html#lambda-expressions-in-java
*/
public class ArmExtend extends Command {

  private Slider m_Slider;
  private double m_setpoint;
  private double goal_tolerance;
  /**
   * Create a new ElevatorGoToPosition command.
   *
   * @param setpoint The setpoint to set the elevator to
   * @param arm The elevator to use
   */
  public void ArmSliderGoToPosition(double setpointInch, double goalTolerance, Slider arm) {
    m_Slider = arm;
    m_setpoint = setpointInch;
    goal_tolerance = goalTolerance;
    addRequirements(m_Slider);
  }

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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmRotate extends Command {

  private Arm m_Arm;
  private double m_setpoint;
  private double goal_tolerance;
  /**
   * Create a new ElevatorGoToPosition command.
   *
   * @param setpoint The setpoint to set the elevator to
   * @param arm The elevator to use
   */
  public void ArmRotateGoToPosition(double setpointInch, double goalTolerance, Arm arm) {
    m_Arm = arm;
    m_setpoint = setpointInch;
    goal_tolerance = goalTolerance;
    addRequirements(m_Arm);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_Arm.setAngleSetPoint(m_setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {

    return m_Arm.atSetpoint(goal_tolerance);
  }
}

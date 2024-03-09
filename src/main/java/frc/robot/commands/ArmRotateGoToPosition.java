package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmRotateGoToPosition extends Command {

  private Arm m_Arm;
  private double m_setpoint;
  private double goal_tolerance;
  /**
   * Create a new ArmGoToPosition command.
   *
   * @param setpoint The setpoint to set the arm to
   * @param arm The arm to use
   */
  public ArmRotateGoToPosition(double angleArmSetPointDegrees, double goalTolerance, Arm arm) {
    m_Arm = arm;
    m_setpoint = angleArmSetPointDegrees;
    goal_tolerance = goalTolerance;
    addRequirements(m_Arm);
  }

  // public void goToIntakeOut(Arm arm) {
  //   m_Arm = arm;
  //   m_setpoint = Constants.ArmSubsystem.armPosOut;
  //   goal_tolerance = Constants.ArmSubsystem.goalTolerance;
  //   addRequirements(m_Arm);
  // }

  // public void goToIntakeIn(Arm arm) {
  //   m_Arm = arm;
  //   m_setpoint = Constants.ArmSubsystem.armPosIn;
  //   goal_tolerance = Constants.ArmSubsystem.goalTolerance;
  //   addRequirements(m_Arm);
  // }

  
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

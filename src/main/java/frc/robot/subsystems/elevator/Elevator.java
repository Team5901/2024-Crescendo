package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private static final double maxLinearVelocityInchPerSec =
      Constants.ElevatorSubsystem.maxLinearVelocityInchPerSec;
  private static final double maxLinearAccelerationInchPerSec =
      Constants.ElevatorSubsystem.maxLinearAccelerationInchPerSec;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(
          maxLinearVelocityInchPerSec, maxLinearAccelerationInchPerSec);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private final ElevatorFeedforward ffModel;

  // private double positionSetPointInch = 0.0;

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    ffModel =
        new ElevatorFeedforward(
            Constants.ElevatorSubsystem.ks,
            Constants.ElevatorSubsystem.kg,
            Constants.ElevatorSubsystem.kv);
    io.configurePID(
        Constants.ElevatorSubsystem.kP,
        Constants.ElevatorSubsystem.kI,
        Constants.ElevatorSubsystem.kD);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("elevator", inputs);

    // Log elevator speed in RPM
    // Logger.getInstance().recordOutput("ElevatorSpeedRPM",
    // getVelocityRPMFromRadsPerSec());
    // Logger.getInstance().recordOutput("ElevatorSetpointInch",
    // inputs.positionSetPointInch);

    // if(Math.abs(m_goal.position-inputs.positionElevatorInch)<0.25 ){
    //   m_goal.position = inputs.positionElevatorInch;
    // }
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(Constants.simLoopPeriodSecs);

    io.setPosition(m_setpoint.position, ffModel.calculate(m_setpoint.velocity));
    Logger.getInstance().recordOutput("ElevatorPosErrorInch", getError());
    Logger.getInstance().recordOutput("elevatorFF", ffModel.calculate(m_setpoint.velocity));
  }

  public void setPositionSetPoint(double positionInch) {
    m_goal = new TrapezoidProfile.State(positionInch, 0);
    // positionSetPointInch = positionInch;

  }

  public double getPosition() {
    return inputs.positionElevatorInch;
    // positionSetPointInch = positionInch;

  }

  public double getError() {
    return Math.abs(inputs.positionElevatorSetPointInch - inputs.positionElevatorInch);
  }

  /** Stops the elevator. */
  public void stop() {
    io.stop();
  }

  public boolean atSetpoint(double goal_tolerance) {
    return ((Math.abs(m_goal.position - inputs.positionElevatorInch)) < goal_tolerance);
  }
  // /** Returns the current velocity in RPM. */
  // public double getVelocityRPMFromRadsPerSec() {
  // return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }

}

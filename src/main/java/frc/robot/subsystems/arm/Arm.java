package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  // UPDATE: Update for arm instead of elevator. Inches or angle?
  private static final double maxVelocityDegreesPerSec =
      Constants.ArmSubsystem.maxVelocityDegreesPerSec;
  private static final double maxAccelerationDegreesPerSec =
      Constants.ArmSubsystem.maxAccelerationDegreesPerSec;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(maxVelocityDegreesPerSec, maxAccelerationDegreesPerSec);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // UPDATE: Figure out how to update this for arm
  private final ElevatorFeedforward ffModel;

  /** Creates a new arm. */
  public Arm(ArmIO io) {
    this.io = io;

    ffModel =
        new ElevatorFeedforward(
            Constants.ElevatorSubsystem.ks,
            Constants.ElevatorSubsystem.kg,
            Constants.ElevatorSubsystem.kv);
    io.configurePID(
        Constants.ArmSubsystem.kP, Constants.ArmSubsystem.kI, Constants.ArmSubsystem.kD);
    SmartDashboard.putNumber("Arm Angle INPUT", inputs.angleArmDegrees); //creates our arm angle input field, updates once
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("arm", inputs);

    var profile = new TrapezoidProfile(m_constraints);
    m_setpoint = profile.calculate(Constants.simLoopPeriodSecs, m_goal, m_setpoint);

    io.setAngle(m_setpoint.position, ffModel.calculate(m_setpoint.velocity));
    Logger.recordOutput("ArmPosErrorInch", getError());
    Logger.recordOutput("armFF", ffModel.calculate(m_setpoint.velocity));
    SmartDashboard.putNumber("Arm Angle",inputs.angleArmDegrees); // adds an arm angle position indicator, for operator's benefit
  }

  // UPDATE: Might need another function to convert from angle to set point inch? Unclear how
  // trapezoid profile worksion
  public void setAngleSetPoint(double angleDegrees) {
    m_goal = new TrapezoidProfile.State(angleDegrees, 0);
  }

  // UPDATE: Angle or inch?
  public double getAngle() {
    return inputs.angleArmDegrees;
  }

  public double getError() {
    return Math.abs(inputs.angleArmSetPointDegrees - inputs.angleArmDegrees);
  }

  /** Stops the elevator. */
  public void stop() {
    io.stop();
  }

  public boolean atSetpoint(double goal_tolerance) {
    return ((Math.abs(m_goal.position - inputs.angleArmDegrees)) < goal_tolerance);
  }

  // public void setAngleAmp() {
  //   setAngleSetPoint(Constants.ArmSubsystem.armPosAmp);
  // }

  // public void setAngleSpeaker() {
  //   setAngleSetPoint(Constants.ArmSubsystem.armPosSpeaker);
  // }

  // public void setAngleGround() {
  //   setAngleSetPoint(Constants.ArmSubsystem.armPosOut);
  // }

  // public void setAngleIntakeIn() {
  //   setAngleSetPoint(Constants.MovementPositions.IntakeInDeg);
  // }

  // public void setAngleTrap() {
  //   setAngleSetPoint(Constants.ArmSubsystem.armPosTrap);
  // }
}

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
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
  private TrapezoidProfile profile;
  private final TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_current = new TrapezoidProfile.State();

  // UPDATE: Figure out how to update this for arm
  private final ArmFeedforward ffModel;

  /** Creates a new arm. */
  public Arm(ArmIO io) {
    this.io = io;
    m_constraints =
        new TrapezoidProfile.Constraints(
            (maxVelocityDegreesPerSec), (maxAccelerationDegreesPerSec));
    profile = new TrapezoidProfile(m_constraints);

    ffModel =
        new ArmFeedforward(
            Constants.ArmSubsystem.ks, Constants.ArmSubsystem.kg, Constants.ArmSubsystem.kv);
    io.configurePID(
        Constants.ArmSubsystem.kP, Constants.ArmSubsystem.kI, Constants.ArmSubsystem.kD);
    SmartDashboard.putNumber(
        "Arm Angle INPUT",
        inputs.angleArmDegrees); // creates our arm angle input field, updates once
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("arm", inputs);

    m_current = profile.calculate(Constants.simLoopPeriodSecs, m_current, m_goal);

    io.setAngle(
        m_current.position,
        ffModel.calculate(Math.toRadians(m_current.position), Math.toRadians(m_current.velocity)));
    Logger.recordOutput("ArmPosErrorInch", getError());

    SmartDashboard.putNumber(
        "Arm Angle",
        inputs.angleArmDegrees); // adds an arm angle position indicator, for operator's benefit
  }

  // UPDATE: Might need another function to convert from angle to set point inch? Unclear how
  // trapezoid profile worksion
  public void setAngleSetPoint(double angleDegrees) {
    m_goal = new TrapezoidProfile.State(angleDegrees, 0);
  }

  public void setVoltage(double volts) {

    io.setVoltage(volts);
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

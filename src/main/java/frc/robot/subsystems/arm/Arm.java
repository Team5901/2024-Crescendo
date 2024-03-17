package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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
  private TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_next = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_last = new TrapezoidProfile.State();
  private DutyCycleEncoder encoder;
  // UPDATE: Figure out how to update this for arm
  private final ArmFeedforward ffModel;

  /** Creates a new arm. */
  public Arm(ArmIO io,DutyCycleEncoder encoder) {
    encoder.reset(); // TODO change this out for a better system soon
    this.io = io;
    this.encoder=encoder;
    m_constraints =
        new TrapezoidProfile.Constraints(
            (maxVelocityDegreesPerSec), (maxAccelerationDegreesPerSec));
    profile = new TrapezoidProfile(m_constraints);

    ffModel =
        new ArmFeedforward(
            Constants.ArmSubsystem.ks,
            Constants.ArmSubsystem.kg,
            Constants.ArmSubsystem.kv,
            Constants.ArmSubsystem.kA);
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
    m_last.position=encoder.getDistance(); // get our current position from our encoder and the previous state's velocity
    m_last.velocity=m_next.velocity;
    m_next = profile.calculate(Constants.simLoopPeriodSecs, m_last, m_goal); // calculate our next point in the trapezoid using our good encoder;

    io.setAngle(
        m_next.position,
        ffModel.calculate(Math.toRadians(m_next.position), Math.toRadians(m_next.velocity)));
    Logger.recordOutput("ArmPosErrorInch", getError());
    Logger.recordOutput("m_goal position", m_goal.position);
    Logger.recordOutput("m_current position", m_next.position);
    SmartDashboard.putNumber(
        "Arm Angle",
        inputs.angleArmDegrees); // adds an arm angle position indicator, for operator's benefit
    SmartDashboard.putNumber("encoder distance",encoder.getDistance());
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
    return encoder.getDistance();
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

  public void setVelocity(double armVelocityRPM) {
    io.setVelocity(
        armVelocityRPM * Constants.ArmSubsystem.gearRatio,
        0.0); // ffModel.calculate(wheelVelocitySetRPM*gearRatio)

    // Log intake setpoint
    Logger.recordOutput("ArmSetpointRPM", armVelocityRPM);
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

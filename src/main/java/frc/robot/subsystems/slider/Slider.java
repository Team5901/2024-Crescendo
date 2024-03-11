package frc.robot.subsystems.slider;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Slider extends SubsystemBase {
  private final SliderIO io;
  private final SliderIOInputsAutoLogged inputs = new SliderIOInputsAutoLogged();
  private static final double maxLinearVelocityInchPerSec =
      Constants.SliderSubsystem.maxLinearVelocityInchPerSec;
  private static final double maxLinearAccelerationInchPerSec =
      Constants.SliderSubsystem.maxLinearAccelerationInchPerSec;

  private TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_current = new TrapezoidProfile.State();
  private TrapezoidProfile profile;
  private final ElevatorFeedforward ffModel;

  /** Creates a new slider. */
  public Slider(SliderIO io) {
    this.io = io;
    m_constraints =
        new TrapezoidProfile.Constraints(
            maxLinearVelocityInchPerSec, maxLinearAccelerationInchPerSec);
    profile = new TrapezoidProfile(m_constraints);

    ffModel =
        new ElevatorFeedforward(
            Constants.SliderSubsystem.ks,
            Constants.SliderSubsystem.kg,
            Constants.SliderSubsystem.kv);
    io.configurePID(
        Constants.SliderSubsystem.kP, Constants.SliderSubsystem.kI, Constants.SliderSubsystem.kD);
    SmartDashboard.putNumber(
        "Slider INPUT",
        inputs.positionSliderInch); // creates an input number area so we can adjust the slider's
    // position at will

    // m_current = profile.calculate(0, m_current, m_goal);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("slider", inputs);

    // Log slider speed in RPM
    // Logger.getInstance().recordOutput("SliderSpeedRPM",
    // getVelocityRPMFromRadsPerSec());

    m_current = profile.calculate(Constants.simLoopPeriodSecs, m_current, m_goal);

    io.setPosition(m_current.position, ffModel.calculate(m_current.velocity));

    Logger.recordOutput("SliderPosErrorInch", getError());
    SmartDashboard.putNumber(
        "Slider Position",
        inputs.positionSliderInch); // constantly updates slider position for the operators benefit
    SmartDashboard.putNumber("Slider SetPoint", m_goal.position);
  }

  public void setPositionSetPoint(double positionSetInch) {
    m_goal = new TrapezoidProfile.State(positionSetInch, 0);
  }

  public double getPosition() {
    return inputs.positionSliderInch;
  }

  public double getError() {
    return Math.abs(inputs.positionSliderSetPointInch - inputs.positionSliderInch);
  }

  /** Stops the slider. */
  public void stop() {
    io.stop();
  }

  public boolean atSetpoint(double goal_tolerance) {
    return ((Math.abs(m_goal.position - inputs.positionSliderInch)) < goal_tolerance);
  }
  // /** Returns the current velocity in RPM. */
  // public double getVelocityRPMFromRadsPerSec() {
  // return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }

}

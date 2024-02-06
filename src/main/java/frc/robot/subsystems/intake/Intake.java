package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private double wheelVelocitySetPointRPM;
  public double wheelVelocityRPM = 0.0;
  public double motorVelocityRPM = 0.0;
  private static final double gearRatio = Constants.IntakeSubsystem.gearRatio;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        ffModel =
            new SimpleMotorFeedforward(Constants.IntakeSubsystem.ks, Constants.IntakeSubsystem.kv);
        io.configurePID(
            Constants.IntakeSubsystem.kP,
            Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);

        break;
      case REPLAY:
        ffModel =
            new SimpleMotorFeedforward(Constants.IntakeSubsystem.ks, Constants.IntakeSubsystem.kv);
        io.configurePID(
            Constants.IntakeSubsystem.kP,
            Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.00);
        io.configurePID(
            Constants.IntakeSubsystem.kP,
            Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);
        break;
      default:
        ffModel =
            new SimpleMotorFeedforward(Constants.IntakeSubsystem.ks, Constants.IntakeSubsystem.kv);
        io.configurePID(
            Constants.IntakeSubsystem.kP,
            Constants.IntakeSubsystem.kI,
            Constants.IntakeSubsystem.kD);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Log intake speed in RPM
    Logger.recordOutput("IntakeSpeedRPM", getVelocityRPM());
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double wheelVelocitySetRPM) {
    wheelVelocitySetPointRPM = wheelVelocitySetRPM;

    io.setVelocity(
        wheelVelocitySetPointRPM * gearRatio,
        0.0); // ffModel.calculate(wheelVelocitySetRPM*gearRatio)

    // Log intake setpoint
    Logger.recordOutput("IntakeSetpointRPM", wheelVelocitySetRPM);
  }

  public void intakeIn() {
    runVelocity(Constants.IntakeSubsystem.intakeInNoteVelRPM);
    // io.setVoltage(Constants.IntakeSubsystem.intakeInNoteVoltage, 0.0);
  }

  public void intakeShoot() {
    runVelocity(Constants.IntakeSubsystem.intakeShootNoteVelRPM);
    // io.setVoltage(Constants.IntakeSubsystem.intakeShootNoteVoltage, 0.0);
  }

  /** Stops the intake. */
  public void holdCurrent() {
    io.setVoltage(Constants.IntakeSubsystem.holdNoteVoltage, 0.0);
    io.setCurrentLimit(Constants.IntakeSubsystem.holdNoteCurrentAmps);
  }

  /** Stops the intake. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return inputs.motorVelocityRPM / gearRatio;
  }
}

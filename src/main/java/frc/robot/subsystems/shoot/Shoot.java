package frc.robot.subsystems.shoot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shoot extends SubsystemBase {
  private final ShootIO io;
  private final ShootIOInputsAutoLogged inputs = new ShootIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private double wheelVelocitySetPointRPM;
  public double wheelVelocityRPM = 0.0;
  public double motorVelocityRPM = 0.0;
  private static final double gearRatio = Constants.ShootSubsystem.gearRatio;

  /** Creates a new Shoot. */
  public Shoot(ShootIO io) {
    this.io = io;
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        ffModel =
            new SimpleMotorFeedforward(Constants.ShootSubsystem.ks, Constants.ShootSubsystem.kv);
        io.configurePID(
            Constants.ShootSubsystem.kP, Constants.ShootSubsystem.kI, Constants.ShootSubsystem.kD);
        break;
      case REPLAY:
        ffModel =
            new SimpleMotorFeedforward(Constants.ShootSubsystem.ks, Constants.ShootSubsystem.kv);
        io.configurePID(
            Constants.ShootSubsystem.kP, Constants.ShootSubsystem.kI, Constants.ShootSubsystem.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.00);
        io.configurePID(
            Constants.ShootSubsystem.kP, Constants.ShootSubsystem.kI, Constants.ShootSubsystem.kD);
        break;
      default:
        ffModel =
            new SimpleMotorFeedforward(Constants.ShootSubsystem.ks, Constants.ShootSubsystem.kv);
        io.configurePID(
            Constants.ShootSubsystem.kP, Constants.ShootSubsystem.kI, Constants.ShootSubsystem.kD);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shoot", inputs);

    // Log intake speed in RPM
    Logger.recordOutput("ShootSpeedRPM", getVelocityRPM());
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double wheelVelocitySetRPM) {
    wheelVelocitySetPointRPM = wheelVelocitySetRPM;

    io.setVelocity(
        wheelVelocitySetPointRPM * gearRatio,
        0.0); // ffModel.calculate(wheelVelocitySetRPM*gearRatio)

    // Log intake setpoint
    Logger.recordOutput("ShootSetpointRPM", wheelVelocitySetRPM);
  }

  public void shootAmp() {
    // UPDATE: Add code for shootAmp
  }

  public void shootSpeaker() {
    // UPDATE: Add code for shootSpeaker
  }

  /** Holds the note. */
  public void holdCurrent() {
    // UPDATE: Add code here
  }

  /** Stops the shoot. */
  public void stop() {
    io.stop();
  }
  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return inputs.motorVelocityRPM / gearRatio;
  }
}

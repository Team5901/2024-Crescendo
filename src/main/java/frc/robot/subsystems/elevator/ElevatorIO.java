package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    // at output shaft
    public double positionElevatorSetPointInch = 0.0;
    public double positionElevatorInch = 0.0;
    public double velocityElevatorInchPerSec = 0.0;
    public double positionMotorSetPointRot = 0.0;
    public double positionMotorShaftRot = 0.0;
    public double velocityMotorRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double positionInch, double ffVolts) {}

  public default void updateState() {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void holdCurrent(int amps) {}

  public default void configurePID(double kP, double kI, double kD) {}
}

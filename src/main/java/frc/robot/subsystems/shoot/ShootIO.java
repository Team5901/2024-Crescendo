package frc.robot.subsystems.shoot;

import org.littletonrobotics.junction.AutoLog;

public interface ShootIO {
  @AutoLog
  public static class ShootIOInputs {
    public double wheelVelocityRPM = 0.0;
    public double motorVelocityRPM = 0.0;
    public double motorVoltageSetPoint = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShootIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPM, double ffVolts) {}

  public default void setVoltage(double voltageSet, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void setCurrentLimit(int amps) {}

  // UPDATE: Update for 2024, determine if setting LEDs is needed here
  // public default void setLEDsPurple() {}
  // public default void setLEDsYellow() {}

  public default void configurePID(double kP, double kI, double kD) {}
}

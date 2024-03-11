package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    // at output shaft
    // UPDATE: Fix variable names for 2024 robot
    public double angleArmSetPointDegrees = 0.0;
    public double angleArmDegrees = 0.0;
    public double velocityAngleArmPerSec = 0.0;
    public double positionMotorSetPointRot = 0.0;
    public double positionMotorShaftRot = 0.0;
    public double velocityMotorRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setAngle(double positionAngle, double ffVolts) {}

  public default void updateState() {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void holdCurrent(int amps) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocity, double ffVolts) {}
}

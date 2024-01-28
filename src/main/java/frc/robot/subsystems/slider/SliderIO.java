package frc.robot.subsystems.slider;

import org.littletonrobotics.junction.AutoLog;

public interface SliderIO {
  @AutoLog
  public static class SliderIOInputs {

    public double positionSliderSetPointInch = 0.0;
    public double positionSliderInch = 0.0;
    public double velocitySliderInchPerSec = 0.0;
    public double positionMotorSetPointRot = 0.0;
    public double positionMotorShaftRot = 0.0;
    public double velocityMotorRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SliderIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double positionInch, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void updateState() {}

  public default void holdCurrent(int amps) {}

  public default void configurePID(double kP, double kI, double kD) {}
}

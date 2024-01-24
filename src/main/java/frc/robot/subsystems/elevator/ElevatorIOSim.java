package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private static final double gearRatio = Constants.ElevatorSubsystem.gearRatio;
  private static final double sprocketDiameterInch = Constants.ElevatorSubsystem.sprocketDiameterInch;
  private static final double sprocketCircumferenceInch = sprocketDiameterInch * Math.PI;
  private static final double sprocketDiameterMeter = Units.inchesToMeters(sprocketDiameterInch);
  private static final double simCarriageWeightKg = Constants.ElevatorSubsystem.simCarriageWeightKg;
  private static final double elevatorSoftLimitUpperMeters = Units
      .inchesToMeters(Constants.ElevatorSubsystem.elevatorSoftLimitUpperInch);
  private static final double elevatorSoftLimitLowerMeters = Units
      .inchesToMeters(Constants.ElevatorSubsystem.elevatorSoftLimitLowerInch);
  private ElevatorSim elevatorSim = new ElevatorSim(
      DCMotor.getNEO(1),
      gearRatio,
      simCarriageWeightKg,
      sprocketDiameterMeter,
      elevatorSoftLimitLowerMeters,
      elevatorSoftLimitUpperMeters,
      true);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private double positionElevatorSetPointInch = 0.0;
  private double positionElevatorInch = 0.0;
  private double velocityElevatorInchPerSec = 0.0;
  private double positionMotorSetPointRot = 0.0;
  private double positionMotorShaftRot = 0.0;
  private double velocityMotorRPM = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    updateState();
    inputs.positionElevatorSetPointInch = positionElevatorSetPointInch;
    inputs.positionElevatorInch = positionElevatorInch;
    inputs.velocityElevatorInchPerSec = velocityElevatorInchPerSec;
    inputs.positionMotorSetPointRot = positionMotorSetPointRot;
    inputs.positionMotorShaftRot = positionMotorShaftRot;
    inputs.velocityMotorRPM = velocityMotorRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    elevatorSim.update(Constants.simLoopPeriodSecs);
  }

  @Override
  public void setPosition(double positionSetInch, double ffVolts) {
    positionElevatorSetPointInch = positionSetInch;
    positionMotorSetPointRot = positionSetInch / (sprocketCircumferenceInch) * gearRatio;
    pid.setSetpoint(positionMotorSetPointRot);
    // double pidout = pid.calculate(positionMotorShaftRot);
    appliedVolts = MathUtil.clamp(
        pid.calculate(positionMotorShaftRot) + ffVolts, -12.0,
        12.0);

    elevatorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void updateState() {
    velocityElevatorInchPerSec = Units.metersToInches(elevatorSim.getVelocityMetersPerSecond());
    positionElevatorInch += velocityElevatorInchPerSec * Constants.simLoopPeriodSecs;
    positionMotorShaftRot = positionElevatorInch / (sprocketCircumferenceInch) * gearRatio;
    velocityMotorRPM = velocityElevatorInchPerSec / (sprocketCircumferenceInch) * gearRatio * 60;
    currentAmps = elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    elevatorSim.setInputVoltage(0.0);
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

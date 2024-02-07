package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// UPDATE: Figure out what to replace this with
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOSim implements ArmIO {
  private static final double gearRatio = Constants.ArmSubsystem.gearRatio;
  private static final double sprocketDiameterInch = Constants.ArmSubsystem.sprocketDiameterInch;
  private static final double sprocketCircumferenceInch = sprocketDiameterInch * Math.PI;
  private static final double sprocketDiameterMeter = Units.inchesToMeters(sprocketDiameterInch);
  private static final double simCarriageWeightKg = Constants.ArmSubsystem.simCarriageWeightKg;
  private static final double armSoftLimitUpperMeters =
      Units.inchesToMeters(Constants.ArmSubsystem.armSoftLimitUpperInch);
  private static final double armSoftLimitLowerMeters =
      Units.inchesToMeters(Constants.ArmSubsystem.armSoftLimitLowerInch);

  private ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          gearRatio,
          simCarriageWeightKg,
          sprocketDiameterMeter,
          armSoftLimitLowerMeters,
          armSoftLimitUpperMeters,
          true,
          0);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  // UPDATE: Update variable names for 2024 robot
  private double positionElevatorSetPointInch = 0.0;
  private double positionElevatorInch = 0.0;
  private double velocityElevatorInchPerSec = 0.0;
  private double positionMotorSetPointRot = 0.0;
  private double positionMotorShaftRot = 0.0;
  private double velocityMotorRPM = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
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
  public void setAngle(double positionSetAngle, double ffVolts) {
    positionElevatorSetPointInch = positionSetAngle;
    positionMotorSetPointRot = positionSetAngle / (sprocketCircumferenceInch) * gearRatio;
    pid.setSetpoint(positionMotorSetPointRot);
    appliedVolts = MathUtil.clamp(pid.calculate(positionMotorShaftRot) + ffVolts, -12.0, 12.0);

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

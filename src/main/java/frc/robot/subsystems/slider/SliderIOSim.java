package frc.robot.subsystems.slider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import frc.robot.Constants;

public class SliderIOSim implements SliderIO {
  private static final double gearRatio = Constants.SliderSubsystem.gearRatio;
  private static final double sprocketDiameterInch = Constants.SliderSubsystem.sprocketDiameterInch;
  private static final double sprocketCircumferenceInch = sprocketDiameterInch * Math.PI;
  private static final double sprocketDiameterMeter = Units.inchesToMeters(sprocketDiameterInch);
  private static final double simCarriageWeightKg = Constants.SliderSubsystem.simCarriageWeightKg;
  private static final double sliderSoftLimitUpperMeters =  Units.inchesToMeters(Constants.SliderSubsystem.sliderSoftLimitUpperInch);
  private static final double sliderSoftLimitLowerMeters =  Units.inchesToMeters(Constants.SliderSubsystem.sliderSoftLimitLowerInch);
  private ElevatorSim sliderSim = new ElevatorSim(
      DCMotor.getNEO(1),
      gearRatio,
      simCarriageWeightKg,
      sprocketDiameterMeter,
      sliderSoftLimitLowerMeters,
      sliderSoftLimitUpperMeters,
      false);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private double positionSliderSetPointInch = 0.0;
  private double positionSliderInch = 0.0;
  private double velocitySliderInchPerSec = 0.0;
  private double positionMotorSetPointRot = 0.0;
  private double positionMotorShaftRot = 0.0;
  private double velocityMotorRPM = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  @Override
  public void updateInputs(SliderIOInputs inputs) {
    updateState();
    inputs.positionSliderSetPointInch = positionSliderSetPointInch;
    inputs.positionSliderInch = positionSliderInch;
    inputs.velocitySliderInchPerSec = velocitySliderInchPerSec;
    inputs.positionMotorSetPointRot = positionMotorSetPointRot;
    inputs.positionMotorShaftRot = positionMotorShaftRot;
    inputs.velocityMotorRPM = velocityMotorRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    sliderSim.update(Constants.simLoopPeriodSecs);
  }

  @Override
  public void setPosition(double positionSetInch, double ffVolts) {
    positionSliderSetPointInch = positionSetInch;
    positionMotorSetPointRot = positionSetInch / (sprocketCircumferenceInch) * gearRatio;
    pid.setSetpoint(positionMotorSetPointRot);
    // double pidout = pid.calculate(positionMotorShaftRot);
    appliedVolts = MathUtil.clamp(
        pid.calculate(positionMotorShaftRot) + ffVolts, -12.0,
        12.0);

    sliderSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void updateState() {
    velocitySliderInchPerSec = Units.metersToInches(sliderSim.getVelocityMetersPerSecond());
    positionSliderInch += velocitySliderInchPerSec * Constants.simLoopPeriodSecs;
    positionMotorShaftRot = positionSliderInch / (sprocketCircumferenceInch) * gearRatio;
    velocityMotorRPM = velocitySliderInchPerSec / (sprocketCircumferenceInch) * gearRatio * 60;
    currentAmps = sliderSim.getCurrentDrawAmps();
  }

  @Override
  public void stop() {
    // appliedVolts = 0.0;
    // sliderSim.setInputVoltage(0.0);
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

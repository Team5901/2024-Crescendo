package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);
  // private static final double gearRatio = Constants.IntakeSubsystem.gearRatio;
  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double motorVelocitySetPointRPM = 0.0;
  public double motorVelocityRPM = 0.0;
  private double motorVoltageSetPoint = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(flywheelSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      flywheelSim.setInputVoltage(appliedVolts);
    }

    flywheelSim.update(0.02);

    // inputs.positionRad = 0.0;
    inputs.motorVelocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.motorVoltageSetPoint = motorVoltageSetPoint;
  }

  @Override
  public void setVelocity(double motorVelocitySetRPM, double ffVolts) {
    closedLoop = true;
    motorVelocitySetPointRPM = motorVelocitySetRPM;
    pid.setSetpoint(motorVelocitySetPointRPM);
    this.ffVolts = ffVolts;
  }

  public void setVoltage(double voltageSet, double ffVolts) {
    motorVoltageSetPoint = voltageSet;
    pid.setSetpoint(motorVoltageSetPoint);
  }

  @Override
  public void setCurrentLimit(int amps) {
    // flywheelSim.setInputVoltage(1.0/12.0);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
    flywheelSim.setInputVoltage(0.0);
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

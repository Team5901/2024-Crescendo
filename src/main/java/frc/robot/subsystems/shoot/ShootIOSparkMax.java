package frc.robot.subsystems.shoot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.subsystems.shoot.ShootIO.ShootIOInputs;

public class ShootIOSparkMax implements ShootIO {
  private final CANSparkMax shootMotor;
  private final RelativeEncoder shootEncoder;

  private final SparkPIDController shootPidController;
  private final Spark lightStrips = new Spark(Constants.ShootSubsystem.LEDsparknumber);
  private double motorVelocitySetPointRPM = 0.0;

  public double motorVelocityRPM = 0.0;

  private double motorVoltageSetPoint = 0.0;

  public ShootIOSparkMax() {
    shootMotor = new CANSparkMax(Constants.ShootSubsystem.deviceID, MotorType.kBrushless);
    shootEncoder = shootMotor.getEncoder();
    shootPidController = shootMotor.getPIDController();
  }

  @Override
  public void updateInputs(ShootIOInputs inputs) {
    inputs.motorVelocityRPM = shootEncoder.getVelocity();
    inputs.appliedVolts = shootMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = shootMotor.getOutputCurrent();
    inputs.motorVoltageSetPoint = motorVoltageSetPoint;
  }

  @Override
  public void setVelocity(double motorVelocitySetRPM, double ffVolts) {
    motorVelocitySetPointRPM = motorVelocitySetRPM;
    shootPidController.setReference(
        motorVelocitySetPointRPM, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  public void setVoltage(double voltageSet, double ffVolts) {
    motorVoltageSetPoint = voltageSet;
    shootPidController.setReference(
        motorVoltageSetPoint, ControlType.kVoltage, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    shootMotor.stopMotor();
  }

  @Override
  public void setCurrentLimit(int amps) {
    // stop();
    shootMotor.setSmartCurrentLimit(amps);
  }

  public void configurePID(double kP, double kI, double kD) {
    shootMotor.restoreFactoryDefaults();
    shootMotor.setInverted(Constants.ShootSubsystem.isInverted);
    shootMotor.enableVoltageCompensation(12.0);
    shootMotor.setSmartCurrentLimit(Constants.ShootSubsystem.maxCurrentAmps);

    shootPidController.setP(kP);
    shootPidController.setI(kI);
    shootPidController.setD(kD);
    shootPidController.setIZone(Constants.ShootSubsystem.kIz);
    shootPidController.setFF(Constants.ShootSubsystem.kFF);
    shootPidController.setOutputRange(
        Constants.ShootSubsystem.kMinOutput, Constants.ShootSubsystem.kMaxOutput);
    shootMotor.setIdleMode(IdleMode.kCoast);
    shootMotor.burnFlash();
  }
}

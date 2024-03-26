package frc.robot.subsystems.shoot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.shoot.ShootIO.ShootIOInputs;

public class ShootIOSparkMax implements ShootIO {
  private final CANSparkMax shootMotor, shootMotor2;
  private final RelativeEncoder shootEncoder, shootEncoder2;
  private final SparkPIDController shootPidController, shootPidController2;
  // private final Spark lightStrips = new Spark(Constants.ShootSubsystem.LEDsparknumber);
  private double motorVelocitySetPointRPM = 0.0;
  private double motorVelocitySetPointRPM2 = 0.0;
  public double motorVelocityRPM = 0.0;

  private double motorVoltageSetPoint = 0.0;

  public ShootIOSparkMax() {
    shootMotor = new CANSparkMax(Constants.ShootSubsystem.deviceID, MotorType.kBrushless);
    shootMotor2 = new CANSparkMax(Constants.ShootSubsystem.deviceID2, MotorType.kBrushless);
    // shootMotor2.follow(shootMotor, Constants.ShootSubsystem.followerInverted);
    shootEncoder = shootMotor.getEncoder();
    shootEncoder2 = shootMotor2.getEncoder();
    shootPidController = shootMotor.getPIDController();
    shootPidController2 = shootMotor2.getPIDController();
    shootMotor.setInverted(!Constants.ShootSubsystem.isInverted);
    shootMotor2.setInverted(!Constants.ShootSubsystem.isInverted);

    // shootMotor.burnFlash();
    // shootMotor2.burnFlash();
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

    motorVelocitySetPointRPM2 = motorVelocitySetRPM * 0.95;
    shootPidController2.setReference(
        motorVelocitySetPointRPM2, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  public void setVoltage(double voltageSet, double ffVolts) {
    motorVoltageSetPoint = voltageSet;
    shootPidController.setReference(
        motorVoltageSetPoint, ControlType.kVoltage, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    shootMotor.stopMotor();
    shootMotor2.stopMotor();
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
    // shootMotor.burnFlash();

    shootMotor2.restoreFactoryDefaults();
    shootMotor2.setInverted(Constants.ShootSubsystem.isInverted);
    shootMotor2.enableVoltageCompensation(12.0);
    shootMotor2.setSmartCurrentLimit(Constants.ShootSubsystem.maxCurrentAmps);

    shootPidController2.setP(kP);
    shootPidController2.setI(kI);
    shootPidController2.setD(kD);
    shootPidController2.setIZone(Constants.ShootSubsystem.kIz);
    shootPidController2.setFF(Constants.ShootSubsystem.kFF);
    shootPidController2.setOutputRange(
        Constants.ShootSubsystem.kMinOutput, Constants.ShootSubsystem.kMaxOutput);
    shootMotor2.setIdleMode(IdleMode.kCoast);
  }
}

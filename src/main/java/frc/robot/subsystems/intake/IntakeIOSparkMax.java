package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  // private static final double gearRatio = Constants.IntakeSubsystem.gearRatio;
  private final CANSparkMax intakeMotor;
  // private final CANSparkMax follower;
  private final RelativeEncoder intakeEncoder;

  private final SparkPIDController intakePidController;
  private final Spark lightStrips = new Spark(3);
  private double motorVelocitySetPointRPM = 0.0;

  public double motorVelocityRPM = 0.0;

  private double motorVoltageSetPoint = 0.0;

  public IntakeIOSparkMax() {
    intakeMotor = new CANSparkMax(Constants.IntakeSubsystem.deviceID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    intakePidController = intakeMotor.getPIDController();

    // follower.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.positionRad = Units.rotationsToRadians(intakeEncoder.getPosition() /
    // gearRatio);
    inputs.motorVelocityRPM = intakeEncoder.getVelocity();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
    inputs.motorVoltageSetPoint = motorVoltageSetPoint;
  }

  @Override
  public void setVelocity(double motorVelocitySetRPM, double ffVolts) {
    motorVelocitySetPointRPM = motorVelocitySetRPM;
    intakePidController.setReference(
        motorVelocitySetPointRPM, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  public void setVoltage(double voltageSet, double ffVolts) {
    motorVoltageSetPoint = voltageSet;
    intakePidController.setReference(
        motorVoltageSetPoint, ControlType.kVoltage, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void setLEDsPurple() {
    lightStrips.set(0.91);
  }

  @Override
  public void setLEDsYellow() {
    lightStrips.set(0.67);
  }

  @Override
  public void setCurrentLimit(int amps) {
    // stop();
    intakeMotor.setSmartCurrentLimit(amps);
  }

  public void configurePID(double kP, double kI, double kD) {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.IntakeSubsystem.isInverted);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(Constants.IntakeSubsystem.maxCurrentAmps);

    intakePidController.setP(kP);
    intakePidController.setI(kI);
    intakePidController.setD(kD);
    intakePidController.setIZone(Constants.IntakeSubsystem.kIz);
    intakePidController.setFF(Constants.IntakeSubsystem.kFF);
    intakePidController.setOutputRange(
        Constants.IntakeSubsystem.kMinOutput, Constants.IntakeSubsystem.kMaxOutput);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
  }
}

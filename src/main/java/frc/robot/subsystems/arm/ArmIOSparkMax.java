package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;
  private final SparkPIDController armPidController;
  private static final double gearRatio = Constants.ArmSubsystem.gearRatio;

  public double angleArmSetPointDegrees = 0.0;
  public double angleArmDegrees = 0.0;
  public double velocityAngleArmPerSec = 0.0;
  public double positionMotorSetPointRot = 0.0;
  public double positionMotorShaftRot = 0.0;
  public double velocityMotorRPM = 0.0;
  public double appliedVolts = 0.0;
  public double currentAmps = 0.0;

  public ArmIOSparkMax() {
    armMotor = new CANSparkMax(Constants.ElevatorSubsystem.deviceID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armPidController = armMotor.getPIDController();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    updateState();
    inputs.angleArmSetPointDegrees = angleArmSetPointDegrees;
    inputs.angleArmDegrees = angleArmDegrees;
    inputs.velocityAngleArmPerSec = velocityAngleArmPerSec;
    inputs.positionMotorSetPointRot = positionMotorSetPointRot;
    inputs.positionMotorShaftRot = positionMotorShaftRot;
    inputs.velocityMotorRPM = velocityMotorRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setAngle(double positionSetAngle, double ffVolts) {
    if (positionSetAngle < 0.5) {
      ffVolts = 0.0;
    }
    if (ffVolts < 0.1) {
      ffVolts = 0.0;
    }
    angleArmSetPointDegrees = positionSetAngle;
    positionMotorSetPointRot = (positionSetAngle / 360) * gearRatio;

    // TODO: Change to speed
    armPidController.setReference(
        positionMotorSetPointRot, ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void updateState() {
    positionMotorShaftRot = armEncoder.getPosition();
    velocityMotorRPM = armEncoder.getVelocity();
    angleArmDegrees = (positionMotorShaftRot / gearRatio) * 360;
    velocityAngleArmPerSec = (velocityMotorRPM / gearRatio) * 360;
    appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    currentAmps = armMotor.getOutputCurrent();
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {

    int smartMotionSlot = 0;
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(Constants.ArmSubsystem.isInverted);
    armMotor.enableVoltageCompensation(12.0);
    armMotor.setSmartCurrentLimit(Constants.ArmSubsystem.maxCurrentAmps);

    armPidController.setP(kP);
    armPidController.setI(kI);
    armPidController.setD(kD);
    armPidController.setOutputRange(
        Constants.ArmSubsystem.kMinOutput, Constants.ArmSubsystem.kMaxOutput);

    armPidController.setSmartMotionMaxVelocity(
        Constants.ArmSubsystem.maxAngularVelocityRPM, smartMotionSlot);
    armPidController.setSmartMotionMinOutputVelocity(
        Constants.ArmSubsystem.minOutputVelocityRPM, smartMotionSlot);
    armPidController.setSmartMotionMaxAccel(
        Constants.ArmSubsystem.maxAngularAccRPMPerSec, smartMotionSlot);

    armMotor.burnFlash();
  }
}

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; // or sparkmaxrelativeencoder?
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final CANSparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;
  private final SparkPIDController elevatorPidController;
  private static final double gearRatio = Constants.ElevatorSubsystem.gearRatio;
  private static final double sprocketDiameterInch =
      Constants.ElevatorSubsystem.sprocketDiameterInch;
  // private static final double elevatorSoftLimitUpper =
  // Constants.ElevatorSubsystem.elevatorSoftLimitUpperInch;
  // private static final double elevatorSoftLimitLower =
  // Constants.ElevatorSubsystem.elevatorSoftLimitLowerInch;
  private static final double sprocketCircumferenceInch = sprocketDiameterInch * Math.PI;

  public double positionElevatorSetPointInch = 0.0;
  public double positionElevatorInch = 0.0;
  public double velocityElevatorInchPerSec = 0.0;
  public double positionMotorSetPointRot = 0.0;
  public double positionMotorShaftRot = 0.0;
  public double velocityMotorRPM = 0.0;
  public double appliedVolts = 0.0;
  public double currentAmps = 0.0;

  public ElevatorIOSparkMax() {
    elevatorMotor = new CANSparkMax(Constants.ElevatorSubsystem.deviceID, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorPidController = elevatorMotor.getPIDController();

    // follower.burnFlash();
  }

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
  }

  @Override
  public void setPosition(double positionSetInch, double ffVolts) {
    if (positionSetInch < 0.5) {
      ffVolts = 0.0;
    }
    if (ffVolts < 0.1) {
      ffVolts = 0.0;
    }
    positionElevatorSetPointInch = positionSetInch;
    positionMotorSetPointRot = positionSetInch / (sprocketCircumferenceInch) * gearRatio;

    elevatorPidController.setReference(
        positionMotorSetPointRot, ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void updateState() {
    positionMotorShaftRot = elevatorEncoder.getPosition();
    velocityMotorRPM = elevatorEncoder.getVelocity();
    positionElevatorInch = positionMotorShaftRot / gearRatio * sprocketCircumferenceInch;
    velocityElevatorInchPerSec = velocityMotorRPM / gearRatio * sprocketCircumferenceInch;
    appliedVolts = elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    currentAmps = elevatorMotor.getOutputCurrent();
  }

  @Override
  public void stop() {
    // maybe unsafe with elevator falling back?
    elevatorMotor.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {

    int smartMotionSlot = 0;
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setInverted(Constants.ElevatorSubsystem.isInverted);
    elevatorMotor.enableVoltageCompensation(12.0);
    elevatorMotor.setSmartCurrentLimit(Constants.ElevatorSubsystem.maxCurrentAmps);

    // elevatorMotor.setSoftLimit( SoftLimitDirection.kReverse, (float)
    // (elevatorSoftLimitLower*gearRatio/(sprocketDiameterInch*Math.PI)));
    // elevatorMotor.setSoftLimit( SoftLimitDirection.kForward, (float)
    // (elevatorSoftLimitUpper*gearRatio/(sprocketDiameterInch*Math.PI)));

    elevatorPidController.setP(kP);
    elevatorPidController.setI(kI);
    elevatorPidController.setD(kD);
    // elevatorPidController.setIZone(Constants.ElevatorSubsystem.kIz);
    // elevatorPidController.setFF(Constants.ElevatorSubsystem.kFF);
    elevatorPidController.setOutputRange(
        Constants.ElevatorSubsystem.kMinOutput, Constants.ElevatorSubsystem.kMaxOutput);

    elevatorPidController.setSmartMotionMaxVelocity(
        Constants.ElevatorSubsystem.maxAngularVelocityRPM, smartMotionSlot);
    elevatorPidController.setSmartMotionMinOutputVelocity(
        Constants.ElevatorSubsystem.minOutputVelocityRPM, smartMotionSlot);
    elevatorPidController.setSmartMotionMaxAccel(
        Constants.ElevatorSubsystem.maxAngularAccRPMPerSec, smartMotionSlot);
    // elevatorPidController.setSmartMotionAllowedClosedLoopError(
    // Constants.ElevatorSubsystem.allowableSmartMotionPosErrorRotations, smartMotionSlot);

    elevatorMotor.burnFlash();
  }
}

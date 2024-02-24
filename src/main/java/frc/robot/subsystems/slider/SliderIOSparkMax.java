package frc.robot.subsystems.slider;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder; // or sparkmaxrelativeencoder?
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SliderIOSparkMax implements SliderIO {
  private final CANSparkMax sliderMotor;
  private final RelativeEncoder sliderEncoder;
  private final SparkPIDController sliderPidController;
  private static final double gearRatio = Constants.SliderSubsystem.gearRatio;
  private static final double sprocketDiameterInch = Constants.SliderSubsystem.sprocketDiameterInch;
  // private static final double sliderSoftLimitUpper =
  // Constants.SliderSubsystem.sliderSoftLimitUpperInch;
  // private static final double sliderSoftLimitLower =
  // Constants.SliderSubsystem.sliderSoftLimitLowerInch;
  private static final double sprocketCircumferenceInch = sprocketDiameterInch * Math.PI;

  public double positionSliderSetPointInch = 0.0;
  public double positionSliderInch = 0.0;
  public double velocitySliderInchPerSec = 0.0;
  public double positionMotorSetPointRot = 0.0;
  public double positionMotorShaftRot = 0.0;
  public double velocityMotorRPM = 0.0;
  public double appliedVolts = 0.0;
  public double currentAmps = 0.0;

  public SliderIOSparkMax() {
    sliderMotor = new CANSparkMax(Constants.SliderSubsystem.deviceID, MotorType.kBrushless);
    sliderEncoder = sliderMotor.getEncoder();
    sliderPidController = sliderMotor.getPIDController();

    // follower.burnFlash();
  }

  // UPDATE: Add function to add follower
  public void addFollower() {}

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
  }

  @Override
  public void setPosition(double positionSetInch, double ffVolts) {
    if (positionSetInch < 0.5) {
      ffVolts = 0.0;
    }
    if (ffVolts < 0.1) {
      ffVolts = 0.0;
    }
    positionSliderSetPointInch = positionSetInch;
    positionMotorSetPointRot = positionSetInch / (sprocketCircumferenceInch) * gearRatio;

    sliderPidController.setReference(
        positionMotorSetPointRot, ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void updateState() {
    positionMotorShaftRot = sliderEncoder.getPosition();
    velocityMotorRPM = sliderEncoder.getVelocity();
    positionSliderInch = positionMotorShaftRot / gearRatio * sprocketCircumferenceInch;
    velocitySliderInchPerSec = velocityMotorRPM / gearRatio * sprocketCircumferenceInch;
    appliedVolts = sliderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    currentAmps = sliderMotor.getOutputCurrent();
  }

  @Override
  public void stop() {
    // maybe unsafe with slider falling back?
    sliderMotor.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {

    int smartMotionSlot = 0;
    sliderMotor.restoreFactoryDefaults();
    sliderMotor.setInverted(Constants.SliderSubsystem.isInverted);
    sliderMotor.enableVoltageCompensation(12.0);
    sliderMotor.setSmartCurrentLimit(Constants.SliderSubsystem.maxCurrentAmps);

    // sliderMotor.setSoftLimit( SoftLimitDirection.kReverse, (float)
    // (sliderSoftLimitLower*gearRatio/(sprocketDiameterInch*Math.PI)));
    // sliderMotor.setSoftLimit( SoftLimitDirection.kForward, (float)
    // (sliderSoftLimitUpper*gearRatio/(sprocketDiameterInch*Math.PI)));

    sliderPidController.setP(kP);
    sliderPidController.setI(kI);
    sliderPidController.setD(kD);
    // sliderPidController.setIZone(Constants.SliderSubsystem.kIz);
    // sliderPidController.setFF(Constants.SliderSubsystem.kFF);
    sliderPidController.setOutputRange(
        Constants.SliderSubsystem.kMinOutput, Constants.SliderSubsystem.kMaxOutput);

    sliderPidController.setSmartMotionMaxVelocity(
        Constants.SliderSubsystem.maxAngularVelocityRPM, smartMotionSlot);
    sliderPidController.setSmartMotionMinOutputVelocity(
        Constants.SliderSubsystem.minOutputVelocityRPM, smartMotionSlot);
    sliderPidController.setSmartMotionMaxAccel(
        Constants.SliderSubsystem.maxAngularAccRPMPerSec, smartMotionSlot);
    // sliderPidController.setSmartMotionAllowedClosedLoopError(
    // Constants.SliderSubsystem.allowableSmartMotionPosErrorRotations, smartMotionSlot);
    sliderMotor.setIdleMode(IdleMode.kCoast);
    sliderMotor.burnFlash();
  }
}

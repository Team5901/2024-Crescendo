package frc.robot.subsystems.slider;



import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.lib.math.Conversions;


public class SliderIOFalcon implements SliderIO {
  private static final double gearRatio = Constants.SliderSubsystem.gearRatio;
  //private static final double sensorResolution = (double)Constants.SliderSubsystem.sensorResolution;
  private static final double sprocketDiameterInch = Constants.SliderSubsystem.sprocketDiameterInch;
  private static final double sprocketCircumferenceInch = sprocketDiameterInch * Math.PI;

   private final WPI_TalonFX  sliderMotor;
  //private final CANSparkMax follower;


  public SliderIOFalcon() {
    sliderMotor= new WPI_TalonFX(Constants.SliderSubsystem.deviceID, "rio"); //change rio to canivore device name 

    //follower.burnFlash();
  }
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

  }

  @Override
  public void setPosition(double positionInch, double ffVolts) { 
    if(ffVolts< 0.1){
      ffVolts = 0.0;
    }
    positionSliderSetPointInch = positionInch;

    double setPointSensorCounts = Conversions.MetersToFalcon(Units.inchesToMeters(positionInch), Units.inchesToMeters(sprocketCircumferenceInch), gearRatio);
    positionMotorSetPointRot = Conversions.falconToDegrees(setPointSensorCounts, gearRatio)/360.0;
    //double setPointSensorCounts = positionInch/(sprocketCircumferenceInch)/Math.PI/gearRatio* sensorResolution*10.0;
    sliderMotor.set(TalonFXControlMode.MotionMagic,setPointSensorCounts);
  }
  
  @Override
  public void updateState() {
    positionMotorShaftRot = Conversions.falconToDegrees(sliderMotor.getSelectedSensorPosition(), gearRatio)/360.0;

    velocityMotorRPM = Conversions.falconToRPM(sliderMotor.getSelectedSensorVelocity(), gearRatio);
    positionSliderInch = positionMotorShaftRot/gearRatio*sprocketCircumferenceInch;
    velocitySliderInchPerSec = velocityMotorRPM/gearRatio*sprocketCircumferenceInch;
    appliedVolts = sliderMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage();
    currentAmps = sliderMotor.getSupplyCurrent();
  }

  @Override
  public void stop() {
    //maybe unsafe with slider falling back out?
    sliderMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    
    sliderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    sliderMotor.configNeutralDeadband(0.04,0);
    sliderMotor.setSensorPhase(false);
    sliderMotor.setInverted(Constants.SliderSubsystem.isInverted);

		sliderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.SliderSubsystem.kTimeoutMs);

		/* Set the peak and nominal outputs */
		sliderMotor.configNominalOutputForward(0, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configNominalOutputReverse(0, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configPeakOutputForward(1, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.configPeakOutputReverse(-1, Constants.SliderSubsystem.kTimeoutMs);

    sliderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,Constants.SliderSubsystem.maxCurrentAmps,Constants.SliderSubsystem.maxCurrentAmps+5.0,1.0));
    //sliderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,Constants.SliderSubsystem.maxCurrentAmps/2.0,20,0.5));

		/* Set Motion Magic gains in slot0 - see documentation */
		sliderMotor.selectProfileSlot(0, 0);
		sliderMotor.config_kF(0, Constants.SliderSubsystem.kFF, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kP(0, kP, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kI(0, kI, Constants.SliderSubsystem.kTimeoutMs);
		sliderMotor.config_kD(0, kD, Constants.SliderSubsystem.kTimeoutMs);

    double CruiseVel = Conversions.MPSToFalcon(Units.inchesToMeters(Constants.SliderSubsystem.maxLinearVelocityInchPerSec), 
    Units.inchesToMeters(sprocketCircumferenceInch), 
    gearRatio); 
    double CruiseAcc = Conversions.MPSToFalcon(Units.inchesToMeters(Constants.SliderSubsystem.maxLinearAccelerationInchPerSec), 
    Units.inchesToMeters(sprocketCircumferenceInch), 
    gearRatio); 

    /* Set acceleration and vcruise velocity - see documentation */
		
		sliderMotor.configMotionAcceleration(CruiseAcc, Constants.SliderSubsystem.kTimeoutMs);
    sliderMotor.configMotionCruiseVelocity(CruiseVel, Constants.SliderSubsystem.kTimeoutMs);
    sliderMotor.configAllowableClosedloopError(0, Constants.SliderSubsystem.allowableSmartMotionPosErrorRotations*2048);
		/* Zero the sensor once on robot boot up */
		sliderMotor.setSelectedSensorPosition(0, 0, Constants.SliderSubsystem.kTimeoutMs);
    
    
  }
}

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// UPDATE: Figure out what to replace this with
// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
// import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOSim implements ArmIO {
  private static final double gearRatio = Constants.ArmSubsystem.gearRatio;

  private static final double armSoftLimitUpperMeters =
      Units.inchesToMeters(Constants.ArmSubsystem.armSoftLimitUpperAngle);
  private static final double armSoftLimitLowerMeters =
      Units.inchesToMeters(Constants.ArmSubsystem.armSoftLimitLowerAngle);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  // UPDATE: Update variable names for 2024 robot
  private double angleArmSetPointDegrees = 0.0;
  private double angleArmDegrees = 0.0;
  private double velocityAngleArmPerSec = 0.0;
  private double positionMotorSetPointRot = 0.0;
  private double positionMotorShaftRot = 0.0;
  private double velocityMotorRPM = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {

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
    angleArmSetPointDegrees = positionSetAngle;
    positionMotorSetPointRot = (positionSetAngle / 360) * gearRatio;
    pid.setSetpoint(positionMotorSetPointRot);
    appliedVolts = MathUtil.clamp(pid.calculate(positionMotorShaftRot) + ffVolts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

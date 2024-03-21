// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.Alert;
import frc.lib.util.Alert.AlertType;
import frc.lib.util.COTSFalconSwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final double simLoopPeriodSecs = 0.02;
  private static final RobotType robot = RobotType.REAL;
  public static final boolean chassisOnly = false;
  public static final String driveCANBUS = "Drivetrain";

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class encoder {
    public static final double encoderOffset = -39.680929;
  }

  public static final class Swerve {
    public static final int pigeonID = 21;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double wheelBase = Units.inchesToMeters(19);
    public static final double trackWidth = Units.inchesToMeters(28.875);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    public static final double wheelRadius = 2 * (0.803 / 1);

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 35;
    public static final int anglePeakCurrentLimit = 45;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 45;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.5;
    public static final double closedLoopRamp = 0.5;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.09 / 12);
    public static final double driveKV = (0.70 / 12);
    public static final double driveKA = (0.3 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 5;
    /** Radians per Second */
    public static final double maxAngularVelocity = (1.25 * 2 * Math.PI); // 1;
    // maximum *decimal*, 0 to 1 throttle to clamp to in swervemodule.java
    public static final double maxOpenLoopThrottle = 0.2;
    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 19;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(216.27);
      // public static final SwerveModuleConstants2 constants =
      //     new SwerveModuleConstants2(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 20;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(173.44);
      // public static final SwerveModuleConstants constants = new SwerveModuleConstants();
    }

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.95);
      // public static final SwerveModuleConstants2 constants =
      //     new SwerveModuleConstants2(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(217.83);
      // public static final SwerveModuleConstants2 constants =
      //     new SwerveModuleConstants2(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class ArmSubsystem {
    public static final int deviceID = 14;
    public static final int deviceID2 = 15;
    public static final boolean isInverted = true;
    public static final boolean followerInverted = true;

    // FeedForward Control
    public static final double ks = 0.00;
    public static final double kv = 1; // 0.2
    public static final double kg = 0.5; // 0.75

    public static final double kP = 0.05;
    public static final double kI = 0.00;
    public static final double kD = 0.0;
    public static final double kA = 0.0;
    public static final double kIz = 0;
    public static final double kFF = 0.0001;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    public static final double gearRatio = 50 * (58 / 12); // Arm chain loop
    // public static final double sprocketDiameterInch = 1.92;

    // Arm details
    public static final double maxVelocityDegreesPerSec = 180.0;
    public static final double maxAccelerationDegreesPerSec = 270.0;
    public static final double armVolts = 6.0;
    // public static final double armVelocityRPM = (1 / 2) * 60;

    // motor shaft details
    public static final int maxCurrentAmps = 30;
    public static final double maxAngularVelocityRPM = (maxVelocityDegreesPerSec / 6) * gearRatio;
    public static final double maxAngularAccRPMPerSec =
        (maxAccelerationDegreesPerSec / 6) * gearRatio;
    public static final double minOutputVelocityRPM = 10.0; // requests below this no voltage output
    public static final double allowableSmartMotionPosErrorRotations = 3.0 * gearRatio;
    public static final double autoPositionErrorInch = 2.0;

    // Degrees
    public static final double armSoftLimitLowerAngle = -15;
    public static final double armPosOut = -10;
    public static final double armPosSpeaker = 15;

    public static final double armPosAmp = 108.0;
    public static final double armPosTrap = 75.0;
    public static final double armPosIn = 5;
    public static final double armSoftLimitUpperAngle = 120.0;
    public static final double goalTolerance = 2;
    public static final double allowableTeleopErrorInch = 1.0;
  }

  // Intake motor
  public static final class IntakeSubsystem {
    // UPDATE: Update deviceID
    public static final int deviceID = 18;
    public static final boolean isInverted = false;
    public static final int LEDsparknumber = 3;

    // FeedForward Control
    public static final double ks = 1;
    public static final double kv = 0.000;
    // Closed Loop Control
    public static final double kP = 0.000000625;
    public static final double kI = 0.000000001;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.0002;
    public static final double kMaxOutput = 100;
    public static final double kMinOutput = -100;

    // NOTE: Double check this value
    public static final double gearRatio = 7;

    public static final int maxCurrentAmps = 60;
    public static final int holdNoteCurrentAmps = 10;
    // Velocity control mode
    public static final double intakeInNoteVelRPM = 250.0;
    public static final double goalToleranceVelocity = 10; // RPM
    public static final double intakeShootNoteVelRPM = 200.0;

    // Voltage control mode
    public static final double holdNoteVoltage = 3.0;

    public static final double intakeInNoteVoltage = 6.0;

    public static final double intakeShootNoteVoltage = 6.0;
  }

  // Shoot motor
  public static final class ShootSubsystem {
    // UPDATE: Update deviceID
    public static final int deviceID = 16;
    public static final int deviceID2 = 17;
    public static final int LEDsparknumber = 4;
    public static final boolean followerInverted = false;
    public static final boolean isInverted = true;

    // FeedForward Control
    public static final double ks = 0.0;
    public static final double kv = 0.000;
    // Closed Loop Control
    public static final double kP = 0.00015;
    public static final double kI = 0.000001;
    public static final double kD = 0.005;
    public static final double kIz = 0;
    public static final double kFF = 0.000180;
    public static final double kMaxOutput = 100;
    public static final double kMinOutput = -100;

    // UPDATE: Update value for 2024 robot
    public static final double gearRatio = 4.0 * 2.0;

    public static final int maxCurrentAmps = 30;
    public static final int holdCurrentAmps = 10;
    // Velocity control mode

    public static final double shootAmpVelRPM = 100.0;
    public static final double goalToleranceVelocity = 1; // Rotations per minute
    public static final double shootSpeakerVelRPM = 300.0;
    // Voltage control mode

    public static final double holdVoltage = 4.0;

    public static final double shootAmpVoltage = 6.0;

    public static final double shootSpeakerVoltage = 7.0;
  }

  public static final class SliderSubsystem {
    public static final int deviceID = 12;
    public static final int deviceID2 = 13;
    public static final int sensorResolution = 2048;
    public static final boolean isInverted = true;
    public static final boolean followerInverted = true;

    // FeedForward Control
    public static final double ks = 0.0;
    public static final double kv = 0.00;
    public static final double kg = 0.00;

    public static final double goalTolerance = 0.1;
    public static final double gearRatio = 15.0;
    public static final double sprocketDiameterInch = 1.29;
    public static final double kP = 0.06;
    public static final double kI = 0.00;
    public static final double kD = 0.0;
    public static final double kIz = 0.0;
    public static final double kFF = 0.0;
    public static final double kMaxOutput = 1.0;
    public static final double kMinOutput = -1.0;
    public static final int kTimeoutMs = 30;

    public static final int maxCurrentAmps = 40;

    public static final double maxAngularVelocityRPM = 1200.0;
    public static final double maxAngularAccRPMPerSec = 1500.0;
    public static final double minOutputVelocityRPM = 20.0; // requests below this no voltage output
    public static final double allowableSmartMotionPosErrorRotations = 1.4 * gearRatio;
    public static final double autoPositionErrorInch = 2.0;

    public static final double maxLinearVelocityInchPerSec = 60;
    public static final double maxLinearAccelerationInchPerSec = 60;
    // Inches
    public static final double sliderSoftLimitLowerInch = 0.0;
    public static final double sliderIntakeIn =
        0.0; // Possibly change to -1 or -2 depending on slack in slider system
    public static final double sliderIntakeOut = 13;
    public static final double sliderAmp = 0.0;
    public static final double sliderSpeaker = 0.0;

    public static final double sliderSoftLimitUpperInch = 15.0;

    public static final double simCarriageWeightKg = 4.0; // ~20 lbs
  }

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.SIM) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.REAL;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case REAL:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case SIM:
        return Mode.SIM;
      default:
        return Mode.REAL;
    }
  }
  // Slider Motor
  // UPDATE: Update slider for 2024

  public static enum RobotType {
    REAL,
    SIM
  }
}

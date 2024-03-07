package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightIONetwork extends SubsystemBase implements LimelightIO {
  NetworkTable table;

  public LimelightIONetwork() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void updateInputs(LimelightIOInputs inputs) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      inputs.botPoseWPI = table.getEntry("botpose_wpired").getDoubleArray(new double[7]);
    } else if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      inputs.botPoseWPI = table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    }
    inputs.tv = table.getEntry("tv").getDouble(0);
    inputs.tx = table.getEntry("tx").getDouble(0);
    inputs.ty = table.getEntry("ty").getDouble(0);
    inputs.tid = table.getEntry("tid").getDouble(0);
    // Botpos transform in field-space (driver station WPILIB origin). Translation (X,Y,Z)
    // Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
  }

  public boolean tagInView(LimelightIOInputs inputs) {
    // updateInputs(inputs);
    SmartDashboard.putBoolean("April Tag in View: ", inputs.tv > 0.0);
    return inputs.tv > 0.0;
  }

  public void moveToCenterOfTag(LimelightIOInputs inputs) {
    // updateInputs(inputs);
    if (inputs.tx > 1) {
      SmartDashboard.putString("Centering April Tag: ", "Move right");
    } else if (inputs.tx < -1) {
      SmartDashboard.putString("Centering April Tag: ", "Move left");
    } else {
      SmartDashboard.putString("Centering April Tag: ", "Centered");
    }
  }

  public void angleToSpeakerTag(LimelightIOInputs inputs) {
    if (inputs.tid == 4 || inputs.tid == 8) {
      SmartDashboard.putNumber("Speaker Tag angle: ", inputs.ty);
    }
  }

  public void tagCenterButton(LimelightIOInputs inputs) {
    if (tagInView(inputs)) {
      moveToCenterOfTag(inputs);
      angleToSpeakerTag(inputs);
    }
  }

  public double alignAprilTag(LimelightIOInputs inputs) {
    if (!tagInView(inputs)) {
      return 0;
    } else {
      return inputs.tx;
    }
  }
}

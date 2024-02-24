package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// Add any constants relating to the field here
public class FieldConstants {
  public static double FieldLength = 16.54; // Meters
  public static double FieldWidth = 8.21; // Meters
  public static double RobotStartingZone =
      1.93; // Distance in meters from the alliance wall to the robot starting zone line
  public static Translation3d kBlueAmpPosition =
      new Translation3d(
          Units.inchesToMeters(72.5), Units.inchesToMeters(323), Units.inchesToMeters(53.38));
  public static Translation3d kRedAmpPosition =
      new Translation3d(
          Units.inchesToMeters(578.77), Units.inchesToMeters(323), Units.inchesToMeters(53.38));
}

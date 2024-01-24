package frc.robot.subsystems.drive;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
    @AutoLog
    public static class LimelightIOInputs{
        public double latency;
        public double[] botPoseWPI;
    }

public default void updateInputs(LimelightIOInputs inputs) {}

}
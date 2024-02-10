package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        public boolean seesTarget = false;
        public double aprilTagId = 0;
        public double tX = 0.0;
        public double tY = 0.0;
        public Pose3d targetPoseRobotSpace;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}

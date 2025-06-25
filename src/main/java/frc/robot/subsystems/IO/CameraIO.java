package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Pose2d;

public interface CameraIO extends AutoCloseable {
    public static class CameraIOInputs implements LoggableInputs {
        public boolean doRejectUpdate = true;
        public Pose2d estimatedPose = new Pose2d();
        public double timestampSeconds = 0.0;
        public double[] stdDevs = new double[]{ 0.1, 0.1, 0.01 };

        public boolean hasTarget = false;
        public int mainAprilTagID = -1;
        public double TX = -1;
        public double TA = 0;

        public void toLog(LogTable table) {
            table.put("DoRejectUpdate", doRejectUpdate);
            table.put("EstimatedPose", estimatedPose);
            table.put("TimestampSeconds", timestampSeconds);
            table.put("StdDevs", stdDevs);

            table.put("HasTarget", hasTarget);
            table.put("MainAprilTagID", mainAprilTagID);
            table.put("TX", TX);
            table.put("TA", TA);
        }

        public void fromLog(LogTable table) {
            doRejectUpdate = table.get("DoRejectUpdate", doRejectUpdate);
            estimatedPose = table.get("EstimatedPose", new Pose2d());
            timestampSeconds = table.get("TimestampSeconds", 0.0);
            stdDevs = table.get("StdDevs", new double[]{ 0.1, 0.1, 0.01 });

            hasTarget = table.get("HasTarget", false);
            mainAprilTagID = table.get("MainAprilTagID", -1);
            TX = table.get("TX", -1);
            TA = table.get("TA", 0);
        }
    }

    public void init();

    public void updateInputs(CameraIOInputs inputs);

    public void periodic();
}

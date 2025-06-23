package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.utils.LimelightHelpers;

public class Limelight {
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry botPosWpiBlue = limelightTable.getEntry("botpose_wpiblue");
    private static int allValidIDs[] = new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};

    private static double angleFromMT1 = 0;

    public static void init() {
        setPipeline(2);
        setConfigurationToLimelight();
    }

    private static void setConfigurationToLimelight() {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", allValidIDs); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("limelight", 2.0f); // Process at half resolution for improved framerate and reduced range
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 
            0.4,   // Forward offset (meters)
            0.0,      // Side offset (meters)
            0.2,        // Height offset (meters)
            0.0,      // Roll (degrees)
            20,      // Pitch (degrees)
            0          // Yaw (degrees)
        );
    }

    public static Object[] update() {
        angleFromMT1 = hasTargetFromReef() ? botPosWpiBlue.getDoubleArray(new Double[]{})[5] : angleFromMT1;

        boolean doRejectUpdate = false;

        double suppliedAngle = SubsystemManager.getDrivebase().getHeading().getDegrees();

        // Temporary replacement due to Pigeon's unpredictability
        if(Limelight.hasTargetFromReef()) suppliedAngle = angleFromMT1;
        
        LimelightHelpers.SetRobotOrientation("limelight", suppliedAngle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] stdDevs = limelightTable.getEntry("stddevs").getDoubleArray(new double[12]);
      
        if (mt2 != null) { // if our angular velocity is greater than 360 degrees per second, ignore vision updates
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                doRejectUpdate = true;
        
            else if(mt2.tagCount == 0)
                doRejectUpdate = true;
        
            else if (hasTarget() == false)
                doRejectUpdate = true;
        
            if(!doRejectUpdate) {
                return new Object[]{ mt2.pose, mt2.timestampSeconds, new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{ stdDevs[6], stdDevs[7], stdDevs[10] }) };
            }
        }
        
        return null;  
    }

    public static boolean hasTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    public static boolean hasTargetFromReef(){
        return getMainAprilTagID() == 6 ||
        getMainAprilTagID() == 7 ||
        getMainAprilTagID() == 8 ||
        getMainAprilTagID() == 9 ||
        getMainAprilTagID() == 10 ||
        getMainAprilTagID() == 11 ||
        getMainAprilTagID() == 17 ||
        getMainAprilTagID() == 18 ||
        getMainAprilTagID() == 19 ||
        getMainAprilTagID() == 20 ||
        getMainAprilTagID() == 21 ||
        getMainAprilTagID() == 22;
    }

    public static double getAngleFromMT1() {
        return angleFromMT1;
    }

    public static Pose2d getVisionPose() {
        double x = botPosWpiBlue.getDoubleArray(new Double[]{})[0];
        double y = botPosWpiBlue.getDoubleArray(new Double[]{})[1];
        double yaw = botPosWpiBlue.getDoubleArray(new Double[]{})[5];
        Pose2d pose = new Pose2d(new Translation2d(x, y), new Rotation2d(yaw));
        return pose;
    }

    public static void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public static int getMainAprilTagID() {
        return (int)LimelightHelpers.getFiducialID("limelight");
    }

    public static double getTX() {
        return LimelightHelpers.getTX("limelight");
    }

    public static double getTA() {
        return LimelightHelpers.getTA("limelight");
    }

    public static boolean filterTargetByTA() {
        return getTA() > 0.15 && hasTarget();
    }
}
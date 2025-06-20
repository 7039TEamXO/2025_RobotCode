package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.utils.LimelightHelpers;

import org.opencv.core.Mat.Tuple2;

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

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public static Tuple2<Pose2d> update() {
        angleFromMT1 = hasTargetFromReef() ? botPosWpiBlue.getDoubleArray(new Double[]{})[5] : angleFromMT1;

        boolean doRejectUpdate = false;

        double suppliedAngle = SubsystemManager.getDrivebase().getHeading().getDegrees();
        
        LimelightHelpers.SetRobotOrientation("limelight", suppliedAngle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      
        if (mt2 != null) {
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI) // if our angular velocity is greater than 360 degrees per second, ignore vision updates
                doRejectUpdate = true;
        
            else if(mt2.tagCount == 0)
                doRejectUpdate = true;
        
            else if (hasTarget() == false)
                doRejectUpdate = true;
        
            if(!doRejectUpdate) {
                return new Tuple2(mt2.pose, new Pose2d(mt2.timestampSeconds,0 ,new Rotation2d(0)));
            }
        }
        
        return null;  
    }

    public static boolean hasTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    public static boolean hasTargetFromReef(){
        return getMainAprilTagId() == 6 ||
        getMainAprilTagId() == 7 ||
        getMainAprilTagId() == 8 ||
        getMainAprilTagId() == 9 ||
        getMainAprilTagId() == 10 ||
        getMainAprilTagId() == 11 ||
        getMainAprilTagId() == 17 ||
        getMainAprilTagId() == 18 ||
        getMainAprilTagId() == 19 ||
        getMainAprilTagId() == 20 ||
        getMainAprilTagId() == 21 ||
        getMainAprilTagId() == 22;
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

    public static void printRobotPose() {
        System.out.println("x: " +  SubsystemManager.getDrivebase().getPose().getX());
        System.out.println("y: " +  SubsystemManager.getDrivebase().getPose().getY());
    }

    public static double getTx() {
        return limelightTable.getEntry("tx").getNumber(0).doubleValue();
    }
    public static double getTy() {
        return limelightTable.getEntry("ty").getNumber(0).doubleValue();
    }

    public static void setPointOfInterest(double offsetX) {
        LimelightHelpers.setFiducial3DOffset("limelight", offsetX, 0, 0);
    }

    public static void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public static int getMainAprilTagId() {
        return (int) LimelightHelpers.getFiducialID("limelight");
    }

    public static void setPriorityTagId(int tagId) {
        LimelightHelpers.setPriorityTagID("limelight", tagId);
    }

    public static void resetPriorityTagId() {
        LimelightHelpers.setPriorityTagID("limelight", -1);    
    }

    public static double getTa() {
        return LimelightHelpers.getTA("limelight");
    }

    public static boolean filterTargetByTa(boolean inAuto) {
        return getTa() > 0.15 && hasTarget();
    }
}
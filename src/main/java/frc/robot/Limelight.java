package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;

import java.lang.reflect.Array;
import java.util.Arrays;

import org.opencv.core.Mat.Tuple2;

public class Limelight {
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static NetworkTableEntry ty = limelightTable.getEntry("ty");
    private static NetworkTableEntry botPosWpiBlue = limelightTable.getEntry("botpose_wpiblue");
    private static int validIDs[] = new int[]{3, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22};
    private static int allValidIDs[] = new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
    
    private static final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        SubsystemManager.getDriveBase().getKinematics(), 
        SubsystemManager.getDriveBase().getHeading(), 
        new SwerveModulePosition[] {
            SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[0].getPosition(),
            SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[1].getPosition(),
            SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[2].getPosition(),
            SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[3].getPosition()
        }, // go through all modules and do .getPosition
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

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
        boolean doRejectUpdate = false;
        double yawRate = SubsystemManager.getDriveBase().getRobotVelocity().omegaRadiansPerSecond;
        double pitch = 20;

        LimelightHelpers.SetRobotOrientation("limelight", (SubsystemManager.getDriveBase().getHeading().getDegrees()), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      
        if (mt2 != null) {
            if(Math.abs(SubsystemManager.getDriveBase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI) // if our angular velocity is greater than 360 degrees per second, ignore vision updates
                doRejectUpdate = true;
        
            else if(mt2.tagCount == 0)
                doRejectUpdate = true;
        
            else if (hasTarget() == false)
                doRejectUpdate = true;
        
            if(!doRejectUpdate) {
                return new Tuple2(mt2.pose, new Pose2d(mt2.timestampSeconds,0 ,new Rotation2d(0)));
                // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                // m_poseEstimator.addVisionMeasurement(
                //     mt2.pose,
                //     mt2.timestampSeconds);
                // updatePosition();
            }
        }
        
        // printRobotPose(); 
        return null;  
    }

    public static boolean hasTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    public static Pose2d getVisionPose() {
        double x = botPosWpiBlue.getDoubleArray(new Double[]{})[0];
        double y = botPosWpiBlue.getDoubleArray(new Double[]{})[1];
        double yaw = botPosWpiBlue.getDoubleArray(new Double[]{})[5];
        Pose2d pose = new Pose2d(new Translation2d(x, y), new Rotation2d(yaw));
        return pose;
    }

    // DEPRECATED
    public static void updatePosition() {
        // MT1 
        if(hasTarget()) {
            if(tx.getDouble(0) != 0 && ty.getDouble(0) != 0){
                if (Array.getLength(botPosWpiBlue.getDoubleArray(new Double[]{})) > 2) {
                    SubsystemManager.getDriveBase().resetOdometry(new Pose2d(
                        botPosWpiBlue.getDoubleArray(new Double[]{})[0],
                        botPosWpiBlue.getDoubleArray(new Double[]{})[1], 
                        SubsystemManager.getDriveBase().getHeading()));  
                }
            }
        }
    }

    // DEPRECATED
    public static void printRobotPose() {
        System.out.println("x: " +  SubsystemManager.getDriveBase().getPose().getX());
        System.out.println("y: " +  SubsystemManager.getDriveBase().getPose().getY());
    }

    public static double getTx() {
        return limelightTable.getEntry("tx").getNumber(0).doubleValue();
    }

    public static void setPointOfInterest(double offsetX){
        double[] array = {0, offsetX, 0};
        LimelightHelpers.setFiducial3DOffset("limelight", offsetX, 0, 0);
    }

    public static void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public static int getMainAprilTagId(){
        return (int) LimelightHelpers.getFiducialID("limelight");
    }

    public static void setPriorityTagId(int tagId){
        System.out.println(tagId);
        LimelightHelpers.setPriorityTagID("limelight", tagId);
    }

    public static void resetPriorityTagId(){
        LimelightHelpers.setPriorityTagID("limelight", -1);    
    }

    public static boolean getTyGreaterThan7(){
        return Math.abs(LimelightHelpers.getTY("limelight")) >= 7;
    }
}
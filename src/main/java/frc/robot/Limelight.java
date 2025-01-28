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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.SubsystemManager;
import java.util.Arrays;

public class Limelight {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry tv = table.getEntry("botpose_wpiblue");
    //private static int[] validIDs = {0, 1, 2, 3, 10};

    public static void update() {
        // get current id for apriltag(get minimal tag)
        // System.out.println("current id : " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("t2d").getDoubleArray(new Double[]{})[9]);
        // LimelightHelpers.SetFiducialIDFiltersOverride("", validIDs);

        // System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]));
        // return true if we see apriltag
        // boolean hasTarget = LimelightHelpers.getTV("limelight"); 
        // System.out.println(hasTarget);

        //System.out.println(SubsystemManager.getDriveBase().getPose().getRotation().getDegrees());

     }

    public static void updatePosition() {
        if(tx.getDouble(0) != 0 && ty.getDouble(0) != 0){
            SubsystemManager.getDriveBase().resetOdometry(new Pose2d(new Translation2d(LimelightHelpers.getBotPose2d_wpiRed("limelight").getX(), LimelightHelpers.getBotPose2d_wpiRed("limelight").getY()), Rotation2d.fromDegrees(LimelightHelpers.getBotPose2d_wpiRed("limelight").getRotation().getDegrees())));   
        }
        System.out.println("X___" + SubsystemManager.getDriveBase().getPose().getX() + " Y___" + SubsystemManager.getDriveBase().getPose().getY() + "  Rot___" + SubsystemManager.getDriveBase().getPose().getRotation());
        
    }

    // private static final SwerveDrivePoseEstimator m_poseEstimator =
    // new SwerveDrivePoseEstimator(
    //     SubsystemManager.getDriveBase().getKinematics(), 
    //     SubsystemManager.getDriveBase().getHeading(), 
    //     new SwerveModulePosition[] {
    //         SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[0].getPosition(),
    //         SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[1].getPosition(),
    //         SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[2].getPosition(),
    //         SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[3].getPosition()
    //     }, // go through all modules and do .getPosition
    //     new Pose2d(),
    //     VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    //     VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    // );

}
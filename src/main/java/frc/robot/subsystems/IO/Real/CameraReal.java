package frc.robot.subsystems.IO.Real;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.IO.CameraIO;
import frc.robot.utils.LimelightHelpers;

public class CameraReal implements CameraIO {
    private final NetworkTable limelightTable;
    private final int allValidIDs[];

    private double angleFromMT1;

    private boolean doRejectUpdate;
    private Pose2d estimatedPose;
    private double timestampSeconds;
    private Matrix<N3, N1> stdDevs;

    public CameraReal() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        allValidIDs = new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};

        angleFromMT1 = 0;

        doRejectUpdate = true;
        estimatedPose = new Pose2d();
        timestampSeconds = 0;
        stdDevs = VecBuilder.fill(0.1, 0.1, 0.01);
    }

    @Override
    public void init() {
        LimelightHelpers.setPipelineIndex("limelight", 2);
        setConfigurationToLimelight();
    }

    private void setConfigurationToLimelight() {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", allValidIDs);
        // Process at half resolution for improved framerate and reduced range
        LimelightHelpers.SetFiducialDownscalingOverride("limelight", 2.0f); 
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 
            0.4,   // Forward offset (meters)
            0.0,      // Side offset (meters)
            0.2,        // Height offset (meters)
            0.0,      // Roll (degrees)
            20,      // Pitch (degrees)
            0          // Yaw (degrees)
        ); // Could in principle borrow from Constants.Limelight.RobotToCamera, but no
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.doRejectUpdate = doRejectUpdate;
        inputs.estimatedPose = estimatedPose;
        inputs.timestampSeconds = timestampSeconds;
        inputs.stdDevs = stdDevs.getData();

        inputs.hasTarget = hasTarget();
        inputs.mainAprilTagID = getMainAprilTagID();
        inputs.TX = LimelightHelpers.getTX("limelight");
        inputs.TA = LimelightHelpers.getTA("limelight");
    }

    @Override
    public void periodic() {
        angleFromMT1 = hasTargetFromReef() ? limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new Double[]{})[5] : angleFromMT1;

        doRejectUpdate = false;

        double suppliedAngle = SubsystemManager.getDrivebase().getHeading().getDegrees();

        // Temporary replacement due to Pigeon's unpredictability
        if(hasTargetFromReef()) suppliedAngle = angleFromMT1;
        
        LimelightHelpers.SetRobotOrientation("limelight", suppliedAngle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] currentStdDevs = limelightTable.getEntry("stddevs").getDoubleArray(new double[12]);
      
        if (mt2 != null) { // if our angular velocity is greater than 360 degrees per second, ignore vision updates
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                doRejectUpdate = true;
        
            else if(mt2.tagCount == 0)
                doRejectUpdate = true;
        
            else if (hasTarget() == false)
                doRejectUpdate = true;
        
            if(!doRejectUpdate) {
                estimatedPose = mt2.pose;
                timestampSeconds = mt2.timestampSeconds;
                stdDevs = VecBuilder.fill(currentStdDevs[6], currentStdDevs[7], currentStdDevs[10]);
            }
        } 
    }

    private boolean hasTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    private boolean hasTargetFromReef() {
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

    private int getMainAprilTagID() {
        return (int)LimelightHelpers.getFiducialID("limelight");
    }

    @Override
    public void close() throws Exception {
        // ...
    }
}

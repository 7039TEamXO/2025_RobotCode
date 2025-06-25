package frc.robot.subsystems.IO.Sim;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.IO.CameraIO;

public class CameraSim implements CameraIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;

    // TUNE!
    public static final Matrix<N3, N1> singleTagStdDevs = Constants.Limelight.CameraProperties.singleTagStdDevs;
    public static final Matrix<N3, N1> multiTagStdDevs = Constants.Limelight.CameraProperties.multiTagStdDevs;

    private boolean doRejectUpdate;
    private Pose2d estimatedPose;
    private Matrix<N3, N1> stdDevs;

    private PhotonTrackedTarget currentTarget = null;

    public CameraSim() {
        camera = new PhotonCamera("camera");
        photonEstimator = new PhotonPoseEstimator(
            Constants.FIELD_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            Constants.Limelight.RobotToCamera);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        visionSim = new VisionSystemSim("camera");
        visionSim.addAprilTags(Constants.FIELD_LAYOUT);

        // Limelight 3G
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(Constants.Limelight.CameraProperties.resWidth, Constants.Limelight.CameraProperties.resHeight, Constants.Limelight.CameraProperties.fovDiag);
        cameraProp.setCalibError(Constants.Limelight.CameraProperties.avgErrorPx, Constants.Limelight.CameraProperties.errorStdDevPx);
        cameraProp.setFPS(Constants.Limelight.CameraProperties.fps);
        cameraProp.setAvgLatencyMs(Constants.Limelight.CameraProperties.avgLatencyMs);
        cameraProp.setLatencyStdDevMs(Constants.Limelight.CameraProperties.latencyStdDevMs);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, Constants.Limelight.RobotToCamera);

        // -------------------------
        doRejectUpdate = true;
        estimatedPose = new Pose2d();
        stdDevs = singleTagStdDevs;
    }

    @Override
    public void init() {
        // ...
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.doRejectUpdate = doRejectUpdate;
        inputs.estimatedPose = estimatedPose;
        inputs.timestampSeconds = Timer.getFPGATimestamp();
        inputs.stdDevs = stdDevs.getData();

        inputs.hasTarget = currentTarget != null;
        inputs.mainAprilTagID = currentTarget != null ? currentTarget.getFiducialId() : -1;
        inputs.TX = currentTarget != null ? currentTarget.getYaw() : -1;
        inputs.TA = currentTarget != null ? currentTarget.getArea() : 0;
    }

    @Override
    public void periodic() {
        visionSim.update(SubsystemManager.getDrivebase().getSimulationPose());

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateStdDevs(visionEst, change.getTargets());

            doRejectUpdate = visionEst.isEmpty();
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                doRejectUpdate = true;

            visionEst.ifPresentOrElse(
                est ->
                    visionSim.getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                () -> {
                    visionSim.getDebugField().getObject("VisionEstimation").setPoses();
                });

            visionEst.ifPresent(est -> {
                estimatedPose = est.estimatedPose.toPose2d();
            });
        }
    }

    private void updateStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            stdDevs = singleTagStdDevs;
        } else {
            var estStdDevs = singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            double minDist = Double.MAX_VALUE;

            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;

                double dist = tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

                if(dist < minDist) {
                    minDist = dist;
                    currentTarget = tgt;
                }

                numTags++;
                avgDist += dist;
            }

            if (numTags == 0) {
                stdDevs = singleTagStdDevs;
                currentTarget = null;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = multiTagStdDevs;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                stdDevs = estStdDevs;
            }
        }
    }

    @Override
    public void close() throws Exception {
        // ...
    }
}

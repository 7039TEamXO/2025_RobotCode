// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

import java.io.File;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.AutoLogOutput;
import org.opencv.core.Mat.Tuple2;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.subsystems.Tray.TrayState;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveDrive swerveDrive;
  private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private static boolean isCloseEnoughToReef = false;
  static Pose2d currentLeftReefPos = new Pose2d(0, 0, new Rotation2d(0));
  static Pose2d currentRightReefPos = new Pose2d(0, 0, new Rotation2d(0));

  public SwerveSubsystem(File directory) {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    // double angleConversionFactor =
    // SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    // In this case the wheel diameter is 4 inches, which must be converted to
    // meters to get meters/second.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    // double driveConversionFactor =
    // SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveDriveConstants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    
    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
                                             // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    setupPathPlanner();
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveDriveConstants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(0), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  // private int counter = 0;
  // public boolean isAuto = false;

  @Override
  public void periodic() {
    isCloseEnoughToReef = Math.abs(driveXPID.getError()) < SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get() &&
        Math.abs(driveYPID.getError()) < SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get() &&
        Math.abs(rotationPID.getError()) < Math.toRadians(SwerveDriveTuning.CLOSE_ANGLE_ERROR_CAP_get()) && 
        !isFarFromReef() &&
        (SubsystemManager.getPSJoystick().L1().getAsBoolean() || 
        SubsystemManager.getPSJoystick().R1().getAsBoolean() ||
        SubsystemManager.getPSJoystick().R3().getAsBoolean());

    if (SubsystemManager.getTrayState() == TrayState.UP) {
      swerveDrive.setMaximumAllowableSpeeds(SwerveDriveTuning.CLIMB_SPEED_get(), SwerveDriveTuning.MIN_ROTATION_V_get());
    }
    else {
      swerveDrive.setMaximumAllowableSpeeds(calculateSpeedAccordingToElevator(SwerveDriveConstants.MAX_SPEED, SwerveDriveTuning.MIN_SPEED_get()),
        calculateSpeedAccordingToElevator(SwerveDriveTuning.MAX_ROTATION_V_get(), SwerveDriveTuning.MIN_ROTATION_V_get()));
    }

    if(Constants.CurrentTuningMode == TuningMode.TUNE) {
      driveXPID.setPID(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get());
      driveYPID.setPID(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get());
      rotationPID.setPID(SwerveDriveTuning.KP_ANGULAR_get(), 0, SwerveDriveTuning.KD_ANGULAR_get());
    }

    // if (isAuto) {
    //   counter++;
    //   Tuple2<Pose2d> tuple = Limelight.update();
    //   if (tuple != null && isRobotVBelowOne(true) && counter > 10) {
    //     Pose2d pos = new Pose2d(tuple.get_0().getX(), tuple.get_0().getY(), getHeading());
    //     double timestampSeconds = tuple.get_1().getX();
    //     swerveDrive.addVisionMeasurement(pos, timestampSeconds);
    //   }
    // }
    // else {
    //   counter = 0;
    Tuple2<Pose2d> tuple = Limelight.update();
    if (tuple != null && Limelight.filterTargetByTA()) {
      Pose2d pos = new Pose2d(tuple.get_0().getX(), tuple.get_0().getY(), getHeading());
      double timestampSeconds = tuple.get_1().getX();

      // Temporary replacement due to Pigeon's unpredictability
      if(Limelight.hasTargetFromReef()) pos = new Pose2d(pos.getX(), pos.getY(), new Rotation2d(Math.toRadians(Limelight.getAngleFromMT1())));
      swerveDrive.addVisionMeasurement(pos, timestampSeconds);
    }
    // }

    swerveDrive.updateOdometry();
    updateClosestReefFace(getPose());

    Logger.recordOutput("Odometry/Pose", getPose());
    Logger.recordOutput("Odometry/RequestedPose", requestedPose);
  }

  @Override
  public void simulationPeriodic() {}

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // final boolean enableFeedforward = true;

      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            // if (enableFeedforward) {
            swerveDrive.drive(
                speedsRobotRelative,
                swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                moduleFeedForwards.linearForces());
            // } else {
              // swerveDrive.setChassisSpeeds(speedsRobotRelative);
            // }
          },

          new PPHolonomicDriveController(
              SwerveDriveConstants.TRANSLATION_PID,
              SwerveDriveConstants.ANGLE_PID
          ),
          config,
          () -> isRedAlliance(),
          this
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  // Terrible quit conditions, not for use!

  // public Command driveToPose(Pose2d pose) {
  //   // Create the constraints to use while pathfinding
  //   PathConstraints constraints = new PathConstraints(
  //       swerveDrive.getMaximumChassisVelocity(), 4.0,
  //       swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

  //   // Since AutoBuilder is configured, we can use it to build pathfinding commands
  //   return AutoBuilder.pathfindToPose(
  //       pose,
  //       constraints,
  //       edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
  //   );
  // }

  private Vector2 getReefCenter() {
    if(isRedAlliance()) return new Vector2(Constants.ReefConstants.REEF_CENTER_X_RED, Constants.ReefConstants.REEF_CENTER_Y_RED);
    else return new Vector2(Constants.ReefConstants.REEF_CENTER_X_BLUE, Constants.ReefConstants.REEF_CENTER_Y_BLUE);
  }

  public boolean isCloseEnoughToReef() {
    return isCloseEnoughToReef;
  }

  public boolean isFarFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY())) > SwerveDriveTuning.FAR_DISTANCE_get();
  }

  public boolean isVeryFarFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY())) > SwerveDriveTuning.VERY_FAR_DISTANCE_get();
  }

  private static double getRotationFactorFromElevator() {
    return Elevator.getCurrentPosition() > 14 ? 0.8 :
      Elevator.getCurrentPosition() > 5 ? 0.8 : 1;
  }

  private PIDController driveXPID = new PIDController(SwerveDriveConstants.KP, 0, SwerveDriveConstants.KD);
  private PIDController driveYPID = new PIDController(SwerveDriveConstants.KP, 0, SwerveDriveConstants.KD);
  private PIDController rotationPID = new PIDController(SwerveDriveConstants.KP_ANGULAR, 0, SwerveDriveConstants.KD_ANGULAR);

  private Pose2d requestedPose;

  public Command driveToPose(Pose2d pose) {
    return run(() -> {
      requestedPose = pose;

      driveXPID.setSetpoint(pose.getX());
      driveYPID.setSetpoint(pose.getY());
      rotationPID.setSetpoint(pose.getRotation().getRadians());
    
      Translation2d freshTranslation = new Translation2d(driveXPID.calculate(getPose().getX()), 
        driveYPID.calculate(getPose().getY()));
      Translation2d scaledTranslation = freshTranslation.times(Math.min(1, 
        swerveDrive.getMaximumChassisVelocity() / freshTranslation.getDistance(Translation2d.kZero)));

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      double angleError = robotAngle - rotationPID.getSetpoint();
      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      swerveDrive.drive(scaledTranslation, rotationPID.calculate(robotAngle) * getRotationFactorFromElevator(), true, false);
    });
  }

  public Command driveToClosestReefPoint(ReefOrientation orientation) {
    switch(orientation) {
      case LEFT:
        return driveToPose(calculateLeftAndRightReefPointsFromTag(getClosestReefFaceRobotPos().getX(), getClosestReefFaceRobotPos().getY(), getClosestReefFaceRobotPos().getRotation().getRadians())[0]);
      case MIDDLE:
        return driveToPose(getClosestReefFaceRobotPos());
      case RIGHT:
        return driveToPose(calculateLeftAndRightReefPointsFromTag(getClosestReefFaceRobotPos().getX(), getClosestReefFaceRobotPos().getY(), getClosestReefFaceRobotPos().getRotation().getRadians())[1]);
      case MIDDLE_FAR:
        return driveToPose(calculateLeftAndRightReefPointsFromTag(getClosestReefFaceRobotPos().getX(), getClosestReefFaceRobotPos().getY(), getClosestReefFaceRobotPos().getRotation().getRadians())[2]);
      case MIDDLE_VERY_FAR:
        return driveToPose(calculateLeftAndRightReefPointsFromTag(getClosestReefFaceRobotPos().getX(), getClosestReefFaceRobotPos().getY(), getClosestReefFaceRobotPos().getRotation().getRadians())[3]);
    }

    return null;
  }

  public Command driveToNet() {
    if (isRedAlliance()) {
      return driveToPose(new Pose2d(
        Constants.FieldConstants.WANTED_X_NET_ALGAE_POS_RED, 
        getPose().getY(), 
        new Rotation2d(Math.toRadians(Constants.FieldConstants.WANTED_ROTATION_ANGLE_NET_ALGAE_POS_RED))
      ));
    }
    else {
      return driveToPose(new Pose2d(
        Constants.FieldConstants.WANTED_X_NET_ALGAE_POS_BLUE, 
        getPose().getY(), 
        new Rotation2d(Math.toRadians(Constants.FieldConstants.WANTED_ROTATION_ANGLE_NET_ALGAE_POS_BLUE))
      ));
    }
  }

  public Command chooseFeeder(final double posY){
    return isRedAlliance() ? 
      (posY < 4 ? driveToRedLeftFeeder() : driveToRedRightFeeder()) :
      (posY < 4 ? driveToBlueRightFeeder() : driveToBlueLeftFeeder());
  }

  public Command driveToFeeder(final double wantedX, final double wantedY, final double wantedAngle) {
    return driveToPose(new Pose2d(wantedX, wantedY, new Rotation2d(Math.toRadians(wantedAngle))));
  }

  private Command driveToRedRightFeeder() {
    return driveToFeeder(Constants.FieldConstants.WANTED_X_FEEDER_RIGHT_RED, 
      Constants.FieldConstants.WANTED_Y_FEEDER_RIGHT_RED, Constants.FieldConstants.WANTED_ROTATION_ANGLE_FEEDER_RIGHT_RED);
  }

  private Command driveToRedLeftFeeder() {
    return driveToFeeder(Constants.FieldConstants.WANTED_X_FEEDER_LEFT_RED, 
      Constants.FieldConstants.WANTED_Y_FEEDER_LEFT_RED, Constants.FieldConstants.WANTED_ROTATION_ANGLE_FEEDER_LEFT_RED);
  }

  private Command driveToBlueRightFeeder() {
    return driveToFeeder(Constants.FieldConstants.WANTED_X_FEEDER_RIGHT_BLUE, 
      Constants.FieldConstants.WANTED_Y_FEEDER_RIGHT_BLUE, Constants.FieldConstants.WANTED_ROTATION_ANGLE_FEEDER_RIGHT_BLUE);
  }

  private Command driveToBlueLeftFeeder() {
    return driveToFeeder(Constants.FieldConstants.WANTED_X_FEEDER_LEFT_BLUE, 
      Constants.FieldConstants.WANTED_Y_FEEDER_LEFT_BLUE, Constants.FieldConstants.WANTED_ROTATION_ANGLE_FEEDER_LEFT_BLUE);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      ChassisSpeeds speeds = swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity());

      // Make the robot move
      driveFieldOriented(speeds);
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8);
      double omegaRadiansPerSecond = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();
      
      // Make the robot move
      swerveDrive.drive(translation, omegaRadiansPerSecond,
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }
  
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        SwerveDriveConstants.MAX_SPEED);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        SwerveDriveConstants.MAX_SPEED);
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  private static int selected_face = -1;
  private static Pose2d closestReefFace = new Pose2d();
  private static Pose2d closestReefFaceRobotPos = new Pose2d();

  private static void updateClosestReefFace(Pose2d currentRobotPose2d) {
    double minDist = Double.MAX_VALUE;
    try {
      var currentAllianceOptional = DriverStation.getAlliance();

    double currentX = currentRobotPose2d.getX();
    double currentY = currentRobotPose2d.getY();

    if (currentAllianceOptional.isPresent()) {

        var currentAlliance = currentAllianceOptional.get();
  
        if (currentAlliance == DriverStation.Alliance.Blue) {
          for (int i = 0; i < Constants.ReefConstants.BLUE_REEF_TAGS_ARRAY.length; i++) {
            int reefFace = Constants.ReefConstants.BLUE_REEF_TAGS_ARRAY[i];
            
            var reefFacePose = fieldLayout.getTagPose(reefFace).get().toPose2d();
            double reefFaceX = reefFacePose.getTranslation().getX();
            double reefFaceY = reefFacePose.getTranslation().getY();
            
            
            double dist = Math.pow(reefFaceX - currentX, 2) + Math.pow(reefFaceY - currentY, 2);
                  
            if (dist < minDist){
              minDist = dist;
              selected_face = reefFace;
              closestReefFace = reefFacePose; 
            }
          }
        }
        else if (currentAlliance == DriverStation.Alliance.Red) {
          for (int i = 0; i < Constants.ReefConstants.RED_REEF_TAGS_ARRAY.length; i++) {
            var reefFace = Constants.ReefConstants.RED_REEF_TAGS_ARRAY[i];
            var reefFacePoseOptional = fieldLayout.getTagPose(reefFace);
            
            if (reefFacePoseOptional.isPresent()) {
              var reefFacePose = reefFacePoseOptional.get().toPose2d();
              double reefFaceX = reefFacePose.getTranslation().getX();
              double reefFaceY = reefFacePose.getTranslation().getY();
              double dist = Math.pow(reefFaceX - currentX, 2) + Math.pow(reefFaceY - currentY, 2);
              
              if (dist < minDist) {
                  minDist = dist;
                  selected_face = reefFace;
                  closestReefFace = reefFacePose;
              }
            }
          }
        }
        }
    } catch (Exception e) {}

    closestReefFaceRobotPos = new Pose2d(
      closestReefFace.getX() + Constants.ReefConstants.M_FROM_TAG_TO_ROBOT * Math.cos(closestReefFace.getRotation().getRadians()),
      closestReefFace.getY() + Constants.ReefConstants.M_FROM_TAG_TO_ROBOT * Math.sin(closestReefFace.getRotation().getRadians()),
      new Rotation2d(closestReefFace.getRotation().getRadians() + Math.PI)
    );
  }

  private static Pose2d getClosestReefFaceRobotPos(){
    return closestReefFaceRobotPos;
  }

  public static int getClosestReefTag() {
    return selected_face;
  }
  
  private static Pose2d[] calculateLeftAndRightReefPointsFromTag(double x, double y, double deg){
    double xR = x + Constants.ReefConstants.M_FROM_TAG_TO_POLES * Math.sin(deg);
    double yR = y - Constants.ReefConstants.M_FROM_TAG_TO_POLES * Math.cos(deg);
    double xL = x - Constants.ReefConstants.M_FROM_TAG_TO_POLES * Math.sin(deg);
    double yL = y + Constants.ReefConstants.M_FROM_TAG_TO_POLES * Math.cos(deg);
    double xM = x - 0.2 * Math.cos(deg);
    double yM = y - 0.2 * Math.sin(deg);
    double xVM = x - 1 * Math.cos(deg);
    double yVM = y - 1 * Math.sin(deg);

    return new Pose2d[] { // check if you can return Pose2d array or need to return normal array containing Pose2d
      new Pose2d(xL, yL, new Rotation2d(deg)),
      new Pose2d(xR, yR, new Rotation2d(deg)),
      new Pose2d(xM, yM, new Rotation2d(deg)),
      new Pose2d(xVM, yVM, new Rotation2d(deg))
    };
  }

  public static double calculateSpeedAccordingToElevator(double maxV, double minV) {
    if (Elevator.getCurrentPosition() <= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
      return maxV;
    }

    return (maxV - minV) -
        ((Elevator.getCurrentPosition() / ElevatorConstants.EncoderMaxPos) * (maxV - minV)) + minV;
  }

  // public static boolean isRobotVBelowOne(boolean inAuto) {
  //   if (inAuto) {
  //     return (Math.abs(swerveDrive.getRobotVelocity().vxMetersPerSecond) < 2) &&
  //       (Math.abs(swerveDrive.getRobotVelocity().vyMetersPerSecond) < 2) &&
  //       ((Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) < 1.5));
  //   } else {
  //     return true;
  //   }
  // }
}

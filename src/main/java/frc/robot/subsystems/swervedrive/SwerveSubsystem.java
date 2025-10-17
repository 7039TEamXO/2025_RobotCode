// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
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
import frc.robot.commands.RedLeftCoralAuto;
import frc.robot.commands.RedRightCoralAuto;
import frc.robot.commands.BlueLeftCoralAuto;
import frc.robot.commands.BlueRightCoralAuto;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import org.dyn4j.geometry.Vector2;
import org.json.simple.parser.ParseException;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.subsystems.Tray.TrayState;
import frc.robot.utils.LocalADStarAK;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveDrive swerveDrive;
  private static SwerveDrive swerveDriveOdometry; // Tuning
  // private static boolean isCloseEnoughToReef = false;

  private static LocalADStarAK localADStarAK = new LocalADStarAK();

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
      swerveDriveOdometry = new SwerveParser(directory).createSwerveDrive(SwerveDriveConstants.MAX_SPEED);
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

    // warm-up?
    localADStarAK.setStartPosition(new Pose2d().getTranslation());
    localADStarAK.setGoalPosition(new Pose2d().getTranslation());
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveDriveConstants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(0), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  // private int counter = 0;
  // public boolean isAuto = false;

  @Override
  public void periodic() {
    // isCloseEnoughToReef = Math.abs(driveXPID.getError()) < SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get() &&
    //     Math.abs(driveYPID.getError()) < SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get() &&
    //     Math.abs(rotationPID.getError()) < Math.toRadians(SwerveDriveTuning.CLOSE_ANGLE_ERROR_CAP_get()) && 
    //     !isFarFromReef() &&
    //     (SubsystemManager.getPSJoystick().L1().getAsBoolean() || 
    //     SubsystemManager.getPSJoystick().R1().getAsBoolean() ||
    //     SubsystemManager.getPSJoystick().R3().getAsBoolean());

    if (SubsystemManager.getTrayState() == TrayState.UP) {
      swerveDrive.setMaximumAllowableSpeeds(SwerveDriveTuning.CLIMB_SPEED_get(), SwerveDriveTuning.MIN_ROTATION_V_get());
    }
    else {
      swerveDrive.setMaximumAllowableSpeeds(calculateSpeedAccordingToElevator(SwerveDriveConstants.MAX_SPEED, SwerveDriveTuning.MIN_SPEED_get()),
        calculateSpeedAccordingToElevator(SwerveDriveTuning.MAX_ROTATION_V_get(), SwerveDriveTuning.MIN_ROTATION_V_get()));
    }

    if(Constants.GetTuningMode() == TuningMode.ACTIVE) {
      driveXPID.setPID(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get());
      driveYPID.setPID(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get());
      rotationPID.setPID(SwerveDriveTuning.KP_ANGULAR_get(), 0, SwerveDriveTuning.KD_ANGULAR_get());

      driveToFeederXPID.setPID(SwerveDriveTuning.KP_FEEDER_get(), 0, SwerveDriveTuning.KD_FEEDER_get());
      driveToFeederYPID.setPID(SwerveDriveTuning.KP_FEEDER_get(), 0, SwerveDriveTuning.KD_FEEDER_get());
      rotationToFeederPID.setPID(SwerveDriveTuning.KP_FEEDER_ANGULAR_get(), 0, SwerveDriveTuning.KD_FEEDER_ANGULAR_get());

      xFilter = LinearFilter.singlePoleIIR(SwerveDriveTuning.FILTER_TIME_CONSTANT_get(), 1);
      yFilter = LinearFilter.singlePoleIIR(SwerveDriveTuning.FILTER_TIME_CONSTANT_get(), 1);
    }

    // if(isAuto) filter by isRobotVBelowOne too
    Object[] tuple = Limelight.update();
    if (tuple != null && (double)tuple[3] > 0.2) {
      swerveDrive.addVisionMeasurement((Pose2d)tuple[0], (double)tuple[1]);
    }

    swerveDrive.updateOdometry();
    swerveDriveOdometry.updateOdometry(); // Tuning
    updateClosestReefFace(getPose());

    Logger.recordOutput("Odometry/Pose", getPose());
    if(Constants.CurrentMode != frc.robot.Constants.Mode.REAL) Logger.recordOutput("Odometry/RealPose", getSimulationPose());
    if(!SubsystemManager.isDriveToPoseActive()) {
      Logger.recordOutput("Odometry/RequestedPose", new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d()));
    }
    Logger.recordOutput("Odometry/BasicPose", swerveDriveOdometry.getPose()); // Tuning
  }

  @Override
  public void simulationPeriodic() {}

  // UNUSED
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            swerveDrive.drive(
                speedsRobotRelative,
                swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                moduleFeedForwards.linearForces());
          },

          new PPHolonomicDriveController(
              new PIDConstants(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get()),
              new PIDConstants(SwerveDriveTuning.KP_ANGULAR_get(), 0, SwerveDriveTuning.KD_ANGULAR_get())
          ),
          config,
          () -> isRedAlliance(),
          this
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public boolean isCloseEnoughToPose(Pose2d pose) {
    return Math.pow(getPose().getX() - pose.getX(), 2) + Math.pow(getPose().getY() - pose.getY(), 2) < Math.pow(SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get(), 2) &&
      Math.abs(getPose().getRotation().getDegrees() - pose.getRotation().getDegrees()) < SwerveDriveTuning.CLOSE_ANGLE_ERROR_CAP_get();
  }

  public Command getAutonomousCommand(String pathName) {
    switch(pathName) {
      default:
      case "Blue Right Coral":
        return new BlueRightCoralAuto();
      case "Blue Left Coral":
        return new BlueLeftCoralAuto();
      case "Red Right Coral":
        return new RedRightCoralAuto();
      case "Red Left Coral":
        return new RedLeftCoralAuto();
    }
}

  PathPlannerTrajectory currentTrajectory = null;
  LinearFilter xFilter = LinearFilter.singlePoleIIR(SwerveDriveConstants.FILTER_TIME_CONSTANT, 1);
  LinearFilter yFilter = LinearFilter.singlePoleIIR(SwerveDriveConstants.FILTER_TIME_CONSTANT, 1);
  
  // TOO UNSTABLE FOR USE [5TH OCT 2025]
  public Command avoidToPose(Pose2d pose) {
    return run(() -> {
      localADStarAK.setStartPosition(getPose().getTranslation());
      localADStarAK.setGoalPosition(pose.getTranslation());

      PathPlannerPath currentPath = localADStarAK.getCurrentPath(
        new PathConstraints(SwerveDriveConstants.MAX_SPEED, 3.0, SwerveDriveTuning.MAX_ROTATION_V_get(), Math.toRadians(720)), 
        new GoalEndState(0, pose.getRotation().rotateBy(Rotation2d.k180deg)));
      
      if(currentPath != null) {
        try {
          currentTrajectory = currentPath.generateTrajectory(getRobotVelocity(), getHeading(), RobotConfig.fromGUISettings());
        } catch (IOException | ParseException e) {
          e.printStackTrace();
        } 
      }

      if(currentTrajectory != null) {
        Logger.recordOutput("Odometry/RequestedPose", pose);

        // Only used to determine direction, not magnitude
        Pose2d targetPose = currentTrajectory.getStates().size() >= 3 ? currentTrajectory.getState(2).pose : pose;

        double slowdownCoefficient = 1;

        driveXPID.setSetpoint(pose.getX());
        driveYPID.setSetpoint(pose.getY());

        Translation2d PIDTranslation = new Translation2d(driveXPID.calculate(getPose().getX()), 
          driveYPID.calculate(getPose().getY()));
        slowdownCoefficient = PIDTranslation.getDistance(Translation2d.kZero) / swerveDrive.getMaximumChassisVelocity();

        double xValue = xFilter.calculate(targetPose.getX() - getPose().getX());
        double yValue = yFilter.calculate(targetPose.getY() - getPose().getY());

        // Cancel smoothing if close enough to the target
        // isCloseEnoughToPose eliminates bounces when we disable and turn the command back on
        if(slowdownCoefficient < 0.5) { 
          xValue = pose.getX() - getPose().getX();
          yValue = pose.getY() - getPose().getY();
        }

        Translation2d freshTranslation = new Translation2d(xValue, yValue);

        Translation2d scaledTranslation = freshTranslation.times(swerveDrive.getMaximumChassisVelocity() * slowdownCoefficient / freshTranslation.getDistance(Translation2d.kZero));

        rotationPID.setSetpoint(pose.getRotation().getRadians());

        double robotAngle = getPose().getRotation().getRadians();

        double angleError = robotAngle - pose.getRotation().getRadians();
        if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
        if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

        swerveDrive.drive(scaledTranslation, rotationPID.calculate(robotAngle) * getRotationFactorFromElevator(), true, false);
      }
    });
  }

  public static Vector2 getReefCenter() {
    if(isRedAlliance()) return new Vector2(Constants.Reef.REEF_CENTER_X_RED, Constants.Reef.REEF_CENTER_Y_RED);
    else return new Vector2(Constants.Reef.REEF_CENTER_X_BLUE, Constants.Reef.REEF_CENTER_Y_BLUE);
  }

  public boolean isFarFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY())) > SwerveDriveTuning.FAR_DISTANCE_get();
  }

  public boolean isVeryFarFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY())) > SwerveDriveTuning.VERY_FAR_DISTANCE_get();
  }
  
  public double getDistanceFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY()));
  }

  public static double getRotationFactorFromElevator() {
    return Elevator.getCurrentPosition() > 14 ? 0.8 :
      Elevator.getCurrentPosition() > 5 ? 0.8 : 1;
  }

  private PIDController driveXPID = new PIDController(SwerveDriveConstants.KP, 0, SwerveDriveConstants.KD);
  private PIDController driveYPID = new PIDController(SwerveDriveConstants.KP, 0, SwerveDriveConstants.KD);
  private PIDController rotationPID = new PIDController(SwerveDriveConstants.KP_ANGULAR, 0, SwerveDriveConstants.KD_ANGULAR);

  private PIDController driveToFeederXPID = new PIDController(SwerveDriveConstants.KP_FEEDER, 0, SwerveDriveConstants.KD_FEEDER);
  private PIDController driveToFeederYPID = new PIDController(SwerveDriveConstants.KP_FEEDER, 0, SwerveDriveConstants.KD_FEEDER);
  private PIDController rotationToFeederPID = new PIDController(SwerveDriveConstants.KP_FEEDER_ANGULAR, 0, SwerveDriveConstants.KD_FEEDER_ANGULAR);

  public Command driveToPose(Pose2d pose, boolean useFeederPID) {
    return run(() -> {
      Logger.recordOutput("Odometry/RequestedPose", pose);

      PIDController X = driveXPID;
      PIDController Y = driveYPID;
      PIDController R = rotationPID;
      if(useFeederPID) {
        X = driveToFeederXPID;
        Y = driveToFeederYPID;
        R = rotationToFeederPID;
      }

      X.setSetpoint(pose.getX());
      Y.setSetpoint(pose.getY());
      R.setSetpoint(pose.getRotation().getRadians());
    
      Translation2d freshTranslation = new Translation2d(X.calculate(getPose().getX()), 
        Y.calculate(getPose().getY()));
      Translation2d scaledTranslation = freshTranslation.times(Math.min(1, 
        swerveDrive.getMaximumChassisVelocity() / freshTranslation.getDistance(Translation2d.kZero)));

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      double angleError = robotAngle - R.getSetpoint();
      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      swerveDrive.drive(scaledTranslation, R.calculate(robotAngle) * getRotationFactorFromElevator(), true, false);
    });
  }

  public Command driveToPose(Pose2d pose) {
    return driveToPose(pose, false);
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
        Constants.Field.WANTED_X_NET_ALGAE_POS_RED, Constants.Field.WANTED_Y_NET_ALGAE_POS_RED, 
        new Rotation2d(Math.toRadians(Constants.Field.WANTED_ROTATION_ANGLE_NET_ALGAE_POS_RED))
      ));
    }
    else {
      return driveToPose(new Pose2d(
        Constants.Field.WANTED_X_NET_ALGAE_POS_BLUE, Constants.Field.WANTED_Y_NET_ALGAE_POS_BLUE, 
        new Rotation2d(Math.toRadians(Constants.Field.WANTED_ROTATION_ANGLE_NET_ALGAE_POS_BLUE))
      ));
    }
  }

  public Command driveToProcessor() {
    if (isRedAlliance()) {
      return driveToPose(new Pose2d(
        Constants.Field.WANTED_X_PROCESSOR_ALGAE_POS_RED, Constants.Field.WANTED_Y_PROCESSOR_ALGAE_POS_RED, 
        new Rotation2d(Math.toRadians(Constants.Field.WANTED_ROTATION_ANGLE_PROCESSOR_ALGAE_POS_RED))
      ));
    }
    else {
      return driveToPose(new Pose2d(
        Constants.Field.WANTED_X_PROCESSOR_ALGAE_POS_BLUE, Constants.Field.WANTED_Y_PROCESSOR_ALGAE_POS_BLUE, 
        new Rotation2d(Math.toRadians(Constants.Field.WANTED_ROTATION_ANGLE_PROCESSOR_ALGAE_POS_BLUE))
      ));
    }
  }

  public Command chooseFeeder(final double posY){
    return isRedAlliance() ? 
      (posY < 4 ? driveToRedLeftFeeder() : driveToRedRightFeeder()) :
      (posY < 4 ? driveToBlueRightFeeder() : driveToBlueLeftFeeder());
  }

  public Command driveToFeeder(final double wantedX, final double wantedY, final double wantedAngle) {
    return driveToPose(new Pose2d(wantedX, wantedY, new Rotation2d(Math.toRadians(wantedAngle))), true);
  }

  private Command driveToRedRightFeeder() {
    return driveToFeeder(Constants.Field.WANTED_X_FEEDER_RIGHT_RED, 
      Constants.Field.WANTED_Y_FEEDER_RIGHT_RED, Constants.Field.WANTED_ROTATION_ANGLE_FEEDER_RIGHT_RED);
  }

  private Command driveToRedLeftFeeder() {
    return driveToFeeder(Constants.Field.WANTED_X_FEEDER_LEFT_RED, 
      Constants.Field.WANTED_Y_FEEDER_LEFT_RED, Constants.Field.WANTED_ROTATION_ANGLE_FEEDER_LEFT_RED);
  }

  private Command driveToBlueRightFeeder() {
    return driveToFeeder(Constants.Field.WANTED_X_FEEDER_RIGHT_BLUE, 
      Constants.Field.WANTED_Y_FEEDER_RIGHT_BLUE, Constants.Field.WANTED_ROTATION_ANGLE_FEEDER_RIGHT_BLUE);
  }

  private Command driveToBlueLeftFeeder() {
    return driveToFeeder(Constants.Field.WANTED_X_FEEDER_LEFT_BLUE, 
      Constants.Field.WANTED_Y_FEEDER_LEFT_BLUE, Constants.Field.WANTED_ROTATION_ANGLE_FEEDER_LEFT_BLUE);
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

  public Pose2d getSimulationPose() {
    return swerveDrive.getSimulationDriveTrainPose().get();
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

  private static boolean isRedAlliance()
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

  public Pose2d getPose() {
    return swerveDrive.getPose();
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

  private int selectedFace = -1;
  private Pose2d closestReefFace = new Pose2d();
  private Pose2d closestReefFaceRobotPos = new Pose2d();

  private void updateClosestReefFace(Pose2d currentRobotPose2d) {
    double minDist = Double.MAX_VALUE;
    try {
      var currentAllianceOptional = DriverStation.getAlliance();

    double currentX = currentRobotPose2d.getX();
    double currentY = currentRobotPose2d.getY();

    if (currentAllianceOptional.isPresent()) {

        var currentAlliance = currentAllianceOptional.get();
  
        if (currentAlliance == DriverStation.Alliance.Blue) {
          for (int i = 0; i < Constants.Reef.BLUE_REEF_TAGS_ARRAY.length; i++) {
            int reefFace = Constants.Reef.BLUE_REEF_TAGS_ARRAY[i];
            
            var reefFacePose = Constants.FIELD_LAYOUT.getTagPose(reefFace).get().toPose2d();
            double reefFaceX = reefFacePose.getTranslation().getX();
            double reefFaceY = reefFacePose.getTranslation().getY();
            
            
            double dist = Math.pow(reefFaceX - currentX, 2) + Math.pow(reefFaceY - currentY, 2);
                  
            if (dist < minDist){
              minDist = dist;
              selectedFace = reefFace;
              closestReefFace = reefFacePose; 
            }
          }
        }
        else if (currentAlliance == DriverStation.Alliance.Red) {
          for (int i = 0; i < Constants.Reef.RED_REEF_TAGS_ARRAY.length; i++) {
            var reefFace = Constants.Reef.RED_REEF_TAGS_ARRAY[i];
            var reefFacePoseOptional = Constants.FIELD_LAYOUT.getTagPose(reefFace);
            
            if (reefFacePoseOptional.isPresent()) {
              var reefFacePose = reefFacePoseOptional.get().toPose2d();
              double reefFaceX = reefFacePose.getTranslation().getX();
              double reefFaceY = reefFacePose.getTranslation().getY();
              double dist = Math.pow(reefFaceX - currentX, 2) + Math.pow(reefFaceY - currentY, 2);
              
              if (dist < minDist) {
                  minDist = dist;
                  selectedFace = reefFace;
                  closestReefFace = reefFacePose;
              }
            }
          }
        }
        }
    } catch (Exception e) {}

    closestReefFaceRobotPos = new Pose2d(
      closestReefFace.getX() + Constants.Reef.M_FROM_TAG_TO_ROBOT * Math.cos(closestReefFace.getRotation().getRadians()),
      closestReefFace.getY() + Constants.Reef.M_FROM_TAG_TO_ROBOT * Math.sin(closestReefFace.getRotation().getRadians()),
      new Rotation2d(closestReefFace.getRotation().getRadians() + Math.PI)
    );
  }

  public Pose2d getClosestReefFaceRobotPos() {
    return closestReefFaceRobotPos;
  }

  public int getClosestReefTag() {
    return selectedFace;
  }
  
  private Pose2d[] calculateLeftAndRightReefPointsFromTag(double x, double y, double deg){
    double xR = x + Constants.Reef.M_FROM_TAG_TO_POLES * Math.sin(deg);
    double yR = y - Constants.Reef.M_FROM_TAG_TO_POLES * Math.cos(deg);
    double xL = x - Constants.Reef.M_FROM_TAG_TO_POLES * Math.sin(deg);
    double yL = y + Constants.Reef.M_FROM_TAG_TO_POLES * Math.cos(deg);
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

  public double calculateSpeedAccordingToElevator(double maxV, double minV) {
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

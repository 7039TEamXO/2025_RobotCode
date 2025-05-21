// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

//import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.lang.System.Logger;
import java.rmi.server.ServerCloneException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.dyn4j.dynamics.TimeStep;
import org.dyn4j.geometry.Vector2;
import org.opencv.core.Mat.Tuple2;
import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Limelight;
import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.Elevator.Elevator;

import frc.robot.subsystems.Tray.TrayState;

public class SwerveSubsystem extends SubsystemBase {

  /**
   * PhotonVision class to keep an accurate odometry.
   */
  // private Vision vision;
  /**
   * Swerve drive object.
   */
  private static SwerveDrive swerveDrive;
  private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private static Pose2d tag_pos = null;
  private static boolean isCloseEnoughToReef = false;
  double currentTagX = 0;
  double currentTagY = 0;
  double currentTagAngle = 0;
  static Pose2d currentLeftReefPos = new Pose2d(0, 0, new Rotation2d(0));
  static Pose2d currentRightReefPos = new Pose2d(0, 0, new Rotation2d(0));

  /**
   * AprilTag field layout.
   */
  // private final AprilTagFieldLayout aprilTagFieldLayout =
  // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  /**
   * Enable vision odometry updates while driving.
   */
  // private final boolean visionDriveTest = false;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
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

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle.
    swerveDrive.setCosineCompensator(false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
                                             // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    SwerveController controller = swerveDrive.getSwerveController();

    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(0), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  private int counter = 0;
  public boolean isAuto = false;

  @Override
  public void periodic() {
    // System.out.println(driveXPID.getError());
    // System.out.println(isCloseEnoughToReef());

    isCloseEnoughToReef = Math.abs(driveXPID.getError()) < SwerveDriveConstants.TEST_DISTANCE_TOLERANCE &&
        Math.abs(driveYPID.getError()) < SwerveDriveConstants.TEST_DISTANCE_TOLERANCE &&
        Math.abs(rotationPID.getError()) < SwerveDriveConstants.TEST_ANGLE_TOLERANCE && 
        !isFarEnoughFromReef() &&
        (SubsystemManager.getpsJoystick().L1().getAsBoolean() || 
        SubsystemManager.getpsJoystick().R1().getAsBoolean() ||
        SubsystemManager.getpsJoystick().R3().getAsBoolean());

    if (SubsystemManager.getTrayState() == TrayState.UP) {
      swerveDrive.setMaximumAllowableSpeeds(Constants.CLIMB_SPEED, Constants.MIN_ROTATION_V);
    }

    else {
      swerveDrive.setMaximumAllowableSpeeds(calculateSpeedAccordingToElevator(Constants.MAX_SPEED, Constants.MIN_SPEED),
        calculateSpeedAccordingToElevator(Constants.MAX_ROTATION_V, Constants.MIN_ROTATION_V));
    }

    if (isAuto) {
      counter++;
      Tuple2<Pose2d> tuple = Limelight.update();
      if (tuple != null && isRobotVBelowOne(true) && counter > 10) {
        Pose2d pos = new Pose2d(tuple.get_0().getX(), tuple.get_0().getY(), SubsystemManager.getDriveBase().getHeading());
        double timestampSeconds = tuple.get_1().getX();
        swerveDrive.addVisionMeasurement(pos, timestampSeconds);
      }
    }
    else {
      counter = 0;
      Tuple2<Pose2d> tuple = Limelight.update();
      if (tuple != null && Limelight.filterTargetByTa(false) && isRobotVBelowOne(false)) {
        Pose2d pos = new Pose2d(tuple.get_0().getX(), tuple.get_0().getY(), SubsystemManager.getDriveBase().getHeading());
        double timestampSeconds = tuple.get_1().getX();
        swerveDrive.addVisionMeasurement(pos, timestampSeconds);
      }
    }

    swerveDrive.updateOdometry();
    updateClosestReefFace(getPose());
  }

  public void updateCloserPoints() {
    tag_pos = getClosestReefFace(swerveDrive.getPose());
    currentTagX = tag_pos.getTranslation().getX();
    currentTagY = tag_pos.getTranslation().getY();
    currentTagAngle = tag_pos.getRotation().getRadians();
  }

  @Override
  public void simulationPeriodic() {}

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              Constants.AutoConstants.TRANSLATION_PID,
              // Translation PID constants
              Constants.AutoConstants.ANGLE_PID
          // Rotation PID constants
          ),
          config,

          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public Command rotateToAngle(double wantedAngle, double tolerance) {
    SwerveController controller = swerveDrive.getSwerveController();
    return run(
        () -> {
          drive(ChassisSpeeds.fromFieldRelativeSpeeds(swerveDrive.getRobotVelocity().vxMetersPerSecond,
              swerveDrive.getRobotVelocity().vyMetersPerSecond,
              3.0 * controller.headingCalculate(getHeading().getRadians(),
                  wantedAngle),
              getHeading()));
        }).until(() -> Math.abs(wantedAngle - getHeading().getRadians()) < tolerance);
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  // terrible quit conditions, not for use

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
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
    if(isRedAlliance()) return new Vector2(13.055, 4.025);
    else return new Vector2(4.485, 4.025);
  }

  public boolean isCloseEnoughToReef() {
    return isCloseEnoughToReef;
  }

  public boolean isFarEnoughFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY())) > 1.7;
  }

  public boolean isVeryFarEnoughFromReef() {
    return getReefCenter().distance(new Vector2(getPose().getX(), getPose().getY())) > 1.8;
  }

  private static double getRotationFactorFromElevator(){
    return Elevator.getCurrentPosition() > 14 ? 0.8 :
    Elevator.getCurrentPosition() > 5 ? 0.8 : 1;
  }

  private PIDController driveXPID = new PIDController(SwerveDriveConstants.TEST_KP, 0, SwerveDriveConstants.TEST_KD);
  private PIDController driveYPID = new PIDController(SwerveDriveConstants.TEST_KP, 0, SwerveDriveConstants.TEST_KD);
  private PIDController rotationPID = new PIDController(SwerveDriveConstants.TEST_KP_ANGULAR, 0, SwerveDriveConstants.TEST_KD_ANGULAR);

  // private Pose2d driveToPoseWanted = new Pose2d(0,0,new Rotation2d(0));

  public Command driveToPose(Pose2d pose) {
    // driveToPoseWanted = pose;
    return run(() -> {
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

  public Command driveToNetScorePos(DoubleSupplier joystickX) {
    if (isRedAlliance()) {
      return driveToPose(new Pose2d(
        SwerveDriveConstants.WANTED_X_NET_ALGAE_POS_RED, 
        getPose().getY(), 
        new Rotation2d(SwerveDriveConstants.WANTED_ROTATION_ANGLE_NET_ALGAE_POS_RED)
      ));
    }
    else {
      return driveToPose(new Pose2d(
        SwerveDriveConstants.WANTED_X_NET_ALGAE_POS_BLUE, 
        getPose().getY(), 
        new Rotation2d(SwerveDriveConstants.WANTED_ROTATION_ANGLE_NET_ALGAE_POS_BLUE)
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
    return driveToFeeder(SwerveDriveConstants.WANTED_X_FEEDER_RIGHT_RED, 
    SwerveDriveConstants.WANTED_Y_FEEDER_RIGHT_RED, SwerveDriveConstants.WANTED_ROTATION_ANGLE_FEEDER_RIGHT_RED);
  }

  private Command driveToRedLeftFeeder() {
    return driveToFeeder(SwerveDriveConstants.WANTED_X_FEEDER_LEFT_RED, 
    SwerveDriveConstants.WANTED_Y_FEEDER_LEFT_RED, SwerveDriveConstants.WANTED_ROTATION_ANGLE_FEEDER_LEFT_RED);
  }

  private Command driveToBlueRightFeeder() {
    return driveToFeeder(SwerveDriveConstants.WANTED_X_FEEDER_RIGHT_BLUE, 
    SwerveDriveConstants.WANTED_Y_FEEDER_RIGHT_BLUE, SwerveDriveConstants.WANTED_ROTATION_ANGLE_FEEDER_RIGHT_BLUE);
  }

  private Command driveToBlueLeftFeeder() {
    return driveToFeeder(SwerveDriveConstants.WANTED_X_FEEDER_LEFT_BLUE, 
    SwerveDriveConstants.WANTED_Y_FEEDER_LEFT_BLUE, SwerveDriveConstants.WANTED_ROTATION_ANGLE_FEEDER_LEFT_BLUE);
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                              translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                              translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                           Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      });
    }
  
    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is towards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
      swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        false); // Open loop is disabled since it shouldn't be used most of the time.
    }
  
    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity)
    {
      swerveDrive.driveFieldOriented(velocity);
    }
  
    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity)
    {
      swerveDrive.drive(velocity);
    }
  
  
    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics()
    {
      return swerveDrive.kinematics;
    }
  
    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
      swerveDrive.resetOdometry(initialHolonomicPose);
    }
  
    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
      return swerveDrive.getPose();
    }
  
    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
      swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
  
    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory)
    {
      swerveDrive.postTrajectory(trajectory);
    }
  
    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro()
    {
      swerveDrive.zeroGyro();
    }
  
    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is available.
     */
    private boolean isRedAlliance()
    {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
  
    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance()
    {
      if (isRedAlliance())
      {
        zeroGyro();
        //Set the pose 180 degrees
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else
      {
        zeroGyro();
      }
    }
  
    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake)
    {
      swerveDrive.setMotorIdleMode(brake);
    }
  
    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading()
    {
      return getPose().getRotation();
    }
  
    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
      Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
      return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                          scaledInputs.getY(),
                                                          headingX,
                                                          headingY,
                                                          getHeading().getRadians(),
                                                          Constants.MAX_SPEED);
    }
  
    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
      Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
  
      return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                          scaledInputs.getY(),
                                                          angle.getRadians(),
                                                          getHeading().getRadians(),
                                                          Constants.MAX_SPEED);
    }
  
    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity()
    {
      return swerveDrive.getFieldVelocity();
    }
  
    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
      return swerveDrive.getRobotVelocity();
    }
  
    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController()
    {
      return swerveDrive.swerveController;
    }
  
    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
      return swerveDrive.swerveDriveConfiguration;
    }
  
    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock()
    {
      swerveDrive.lockPose();
    }
  
    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch()
    {
      return swerveDrive.getPitch();
    }
  
    /**
     * Add a fake vision reading for testing purposes.
     */
    // public void addFakeVisionReading()
    // {
    //   swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.frPomDegrees(65)), Timer.getFPGATimestamp());
    // }
  
    public void print() {
      // System.out.println(swerveDrive.getPose().getX());
    }

    private static int selected_face = -1;
    private static Pose2d closestReefFace = null;
    private static Pose2d closestReefFaceRobotPos = null;

    private static void updateClosestReefFace(Pose2d currentRobotPose2d){
      double minDist = Double.MAX_VALUE;
      try {
        var currentAllianceOptional = DriverStation.getAlliance();

      double currentX = currentRobotPose2d.getX();
      double currentY = currentRobotPose2d.getY();

      if (currentAllianceOptional.isPresent()) {

          var currentAlliance = currentAllianceOptional.get();
    
          if (currentAlliance == DriverStation.Alliance.Blue) {
            for (int i = 0; i < SwerveDriveConstants.BLUE_RIFF_TAGS_ARRAY.length; i++) {
              int reefFace = SwerveDriveConstants.BLUE_RIFF_TAGS_ARRAY[i];
              
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
            for (int i = 0; i < SwerveDriveConstants.RED_RIFF_TAGS_ARRAY.length; i++) {
              var reefFace = SwerveDriveConstants.RED_RIFF_TAGS_ARRAY[i];
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
      } catch (Exception e){
      }
      // System.out.println(selected_face);
      // System.out.println("TAG: " + selected_face);
      closestReefFaceRobotPos = new Pose2d(
        closestReefFace.getX() + 0.51 * Math.cos(closestReefFace.getRotation().getRadians()),
        closestReefFace.getY() + 0.51 * Math.sin(closestReefFace.getRotation().getRadians()),
        new Rotation2d(closestReefFace.getRotation().getRadians() + Math.PI)
      );

      // System.out.println(closestReefFace);
      // System.out.println("   " + closestReefFaceRobotPos);
    }

  
    private static Pose2d getClosestReefFace(Pose2d currentRobotPose2d){
      return closestReefFace;
    }

    private static Pose2d getClosestReefFaceRobotPos(){
      return closestReefFaceRobotPos;
    }


    public static int getClosestReefTag() {
      return selected_face;
    }
  
    public static double convertDegToRag(double deg){
      if(deg > 360) {
        return 0;
      }
      return deg * Constants.DEG_TO_RAD;
    }
  
    public static double convertRadToDeg(double rad){
      if(rad > Math.PI * 2) {
        return 0;
      }
      return rad / Constants.DEG_TO_RAD;
    }
  
  private static Pose2d[] calculateLeftAndRightReefPointsFromTag(double x, double y, double deg){
    double xR = x + SwerveDriveConstants.M_FROM_TAG_TO_POLES * Math.sin(deg);
    double yR = y - SwerveDriveConstants.M_FROM_TAG_TO_POLES * Math.cos(deg);
    double xL = x - SwerveDriveConstants.M_FROM_TAG_TO_POLES * Math.sin(deg);
    double yL = y + SwerveDriveConstants.M_FROM_TAG_TO_POLES * Math.cos(deg);
    double xM = x - 0.2 * Math.cos(deg);
    double yM = y - 0.2 * Math.sin(deg);
    double xVM = x - 1 * Math.cos(deg);
    double yVM = y - 1 * Math.sin(deg);

    // System.out.println("XR: " + xR + " YR: " + yR);
    // System.out.println("XL: " + xL + " YL: " + yL);

    return new Pose2d[] { // check if you can return pose2d array or need to return normal array
                        // containing pose2d
      new Pose2d(xL, yL, new Rotation2d(deg)),
      new Pose2d(xR, yR, new Rotation2d(deg)),
      new Pose2d(xM, yM, new Rotation2d(deg)),
      new Pose2d(xVM, yVM, new Rotation2d(deg))
    };
  }

  public Pose2d getcurrentRightReefPos() {
    return currentRightReefPos;
  }

  public Pose2d getcurrentLeftReefPos() {
    return currentLeftReefPos;
  }

  public static Pose2d getCurrentAprilTagPos() {
    return tag_pos;
  }

  public static boolean teamColorIsBlue() {
    Optional<Alliance> color = DriverStation.getAlliance();
    return color.get() == DriverStation.Alliance.Blue;
  }

  public static double calculateSpeedAccordingToElevator(double maxV, double minV) {
    if (Elevator.getCurrentPosition() <= ElevatorConstants.ELEVATOR_POSE_SAFE_TO_ROTATE) {
      return maxV;
    }

    return (maxV - minV) -
        ((Elevator.getCurrentPosition() / ElevatorConstants.maxPos) * (maxV - minV)) + minV;

  }

  public static boolean isRobotVBelowOne(boolean inAuto) {
    if (inAuto) {
      return (Math.abs(swerveDrive.getRobotVelocity().vxMetersPerSecond) < 2) &&
        (Math.abs(swerveDrive.getRobotVelocity().vyMetersPerSecond) < 2) &&
        ((Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) < 1.5));
    }else{
      return true;
      // return (Math.abs(swerveDrive.getRobotVelocity().vxMetersPerSecond) < 3) &&
        // (Math.abs(swerveDrive.getRobotVelocity().vyMetersPerSecond) < 3) &&
        // ((Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) < 2));
    }
  }

  public double getAngleFromCurrentTag() {
    try {
      // int tagId = Limelight.getMainAprilTagId();
      int tagId = getClosestReefTag();
      double tagRot = fieldLayout.getTagPose(tagId).get().toPose2d().getRotation().getDegrees();

      double flippedDeg = (tagRot + 180) % 360;

      if (flippedDeg > 180) {
        flippedDeg -= 360;
      }

      double currentHeading = swerveDrive.getPose().getRotation().getDegrees();
      double angleDiff = currentHeading - flippedDeg;

      if (angleDiff > 180) {
        angleDiff -= 360;
      } else if (angleDiff < -180) {
        angleDiff += 360;
      }

      return (angleDiff);

    } catch (Exception e) {
      return 0;
    }

  }
  

  private static double getAngleToNet(double wantedAngle){
    try{
    double currentHeading = swerveDrive.getOdometryHeading().getDegrees();
    double angleDiff = currentHeading - wantedAngle;

    if (angleDiff > 180) {
      angleDiff -= 360;
    } else if (angleDiff < -180) {
      angleDiff += 360;
    }

    return (angleDiff);

  } catch (Exception e) {
    return 0;
  }
  }

}

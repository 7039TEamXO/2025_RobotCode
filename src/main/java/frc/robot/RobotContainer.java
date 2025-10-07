package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.SubsystemManager;
// import frc.robot.subsystems.Handler.Handler;
import frc.robot.subsystems.SwerveDrive.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.SwerveDriveTuning;
// import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

import java.util.Optional;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class RobotContainer {
  Command driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftY(), Operator.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftX(), Operator.LEFT_X_DEADBAND),
      () -> -SubsystemManager.getPSJoystick().getRightX());
      
  SlewRateLimiter joystickSlewRateLimiter = new SlewRateLimiter(4);

  public boolean driveCommandConfigured = false;

  public RobotContainer() {
    configureBindings();
  }

  public void updateDriveCommand() {
    rotationPID.setPID(SwerveDriveTuning.KP_ALIGN_get(), 0, SwerveDriveTuning.KD_ALIGN_get());
    configureDriveCommand();
  }

  private void configureBindings() {    
    SubsystemManager.getPSJoystick().PS().onTrue((Commands.runOnce(SubsystemManager.getDrivebase()::zeroGyroWithAlliance)));
      // to use in case the rotation is messed up
  }

  private PIDController rotationPID = new PIDController(SwerveDriveConstants.KP_ALIGN, 0, SwerveDriveConstants.KD_ALIGN);

  private double alignAtReef() {
    /*
    if(Handler.isCoralIn() || Handler.getCoralIR()) {
      Pose2d requestedPose = SubsystemManager.getDrivebase().getPose();

      requestedPose = new Pose2d(requestedPose.getX(), requestedPose.getY(), SubsystemManager.getDrivebase().getClosestReefFaceRobotPos().getRotation());
      Logger.recordOutput("Odometry/RequestedPose", requestedPose);

      rotationPID.setSetpoint(requestedPose.getRotation().getRadians());

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = SubsystemManager.getDrivebase().getPose().getRotation().getRadians();

      double angleError = robotAngle - rotationPID.getSetpoint();
      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      return rotationPID.calculate(robotAngle) * SwerveSubsystem.getRotationFactorFromElevator();
    } else {
    */
      return modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND);
    //}
  }

  private void configureDriveCommand() {
    if (teamColorIsBlue() == 0) {
      SubsystemManager.getDrivebase().zeroGyro();
      SubsystemManager.getDrivebase().resetOdometry(new Pose2d(SubsystemManager.getDrivebase().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> alignAtReef());

        driveCommandConfigured = true;
    } else if(teamColorIsBlue() == 1) {
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> alignAtReef());

        driveCommandConfigured = true;
    }
    
    SubsystemManager.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  public Command getAutonomousCommand() {
    return SubsystemManager.getDrivebase().getAutonomousCommand(Dashboard.getSelectedAutonomy());
  }

  public void setDriveMode()
  {}

  public void setMotorBrake(boolean brake) {
    SubsystemManager.getDrivebase().setMotorBrake(brake);
  }

  public static int teamColorIsBlue() {
    try {
      Optional<Alliance> color = DriverStation.getAlliance();
	    return color.get() == DriverStation.Alliance.Blue ? 1 : 0;
    } catch (Exception e) {
      return -1;
    }
  }

  private static double modifyAxis(double joystickInput, double deadband) {
    double deadbandValue = MathUtil.applyDeadband(joystickInput, deadband);
    double scaled_input = 1 - (1 - Math.abs(deadbandValue)) * (1 - 0.1);
    return deadbandValue * scaled_input;
  }
}

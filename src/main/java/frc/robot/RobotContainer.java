package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.SubsystemManager;

import java.util.Optional;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class RobotContainer {
  Command driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftY(), Operator.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftX(), Operator.LEFT_X_DEADBAND),
      () -> -SubsystemManager.getPSJoystick().getRightX());
      
  SlewRateLimiter joystickSlewRateLimiter = new SlewRateLimiter(4);

  public RobotContainer() {
    configureBindings();
    configureDriveCommand();
  }

  private void configureBindings() {    
    SubsystemManager.getPSJoystick().PS().onTrue((Commands.runOnce(SubsystemManager.getDrivebase()::zeroGyroWithAlliance)));
      // to use in case the rotation is messed up
  }

  private void configureDriveCommand() {
    if (teamColorIsBlue()) {
      SubsystemManager.getDrivebase().zeroGyro();
      SubsystemManager.getDrivebase().resetOdometry(new Pose2d(SubsystemManager.getDrivebase().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND)));
    } else {
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND)));
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

  public static boolean teamColorIsBlue() {
    try {
      Optional<Alliance> color = DriverStation.getAlliance();
	    return color.get() == DriverStation.Alliance.Blue;
    } catch (Exception e) {
      return true;
    }
  }

  private static double modifyAxis(double joystickInput, double deadband) {
    double deadbandValue = MathUtil.applyDeadband(joystickInput, deadband);
    double scaled_input = 1 - (1 - Math.abs(deadbandValue)) * (1 - 0.1);
    return deadbandValue * scaled_input;
  }
}

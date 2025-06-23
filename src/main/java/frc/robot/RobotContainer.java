package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SubsystemManager;

import java.util.Optional;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  Command driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-SubsystemManager.getPSJoystick().getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -SubsystemManager.getPSJoystick().getRightX());
      
  SlewRateLimiter joystickSlewRateLimiter = new SlewRateLimiter(4);

  public RobotContainer() {
    configureBindings();
    configureDriveCommand();

    NamedCommands.registerCommand("Travel", SubsystemManager.travelCommand);
    NamedCommands.registerCommand("IntakeCoral", SubsystemManager.intakeCoralCommand);
    NamedCommands.registerCommand("IntakeAlgaeLow", SubsystemManager.intakeAlgaeLowCommand);
    NamedCommands.registerCommand("IntakeAlgaeHigh", SubsystemManager.intakeAlgaeHighCommand);
    NamedCommands.registerCommand("Base", SubsystemManager.baseCommand);
    NamedCommands.registerCommand("Level_0", SubsystemManager.level0Command);
    NamedCommands.registerCommand("Level_1", SubsystemManager.level1Command);
    NamedCommands.registerCommand("Level_2", SubsystemManager.level2Command);
    NamedCommands.registerCommand("Level_3", SubsystemManager.level3Command);
    NamedCommands.registerCommand("Deplete", SubsystemManager.depleteCommand);
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
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)));
    } else {
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)));
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

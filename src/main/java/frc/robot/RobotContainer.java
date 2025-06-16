package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SubsystemManager;

import java.util.Optional;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer
{
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // final CommandPS4Controller SubsystemManager.ps4Joystick = new CommandPS4Controller(0);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(SubsystemManager.getDriveBase(),
  //                                                                () -> -MathUtil.applyDeadband(SubsystemManager.ps4Joystick.getLeftY(),
  //                                                                                              OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(SubsystemManager.ps4Joystick.getLeftX(),
  //                                                                                              OperatorConstants.LEFT_X_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(-SubsystemManager.ps4Joystick.getRightX(),
  //                                                                                              OperatorConstants.RIGHT_X_DEADBAND),
  //                                                                SubsystemManager.ps4Joystick.povDown(),
  //                                                                SubsystemManager.ps4Joystick.povUp(),
  //                                                                SubsystemManager.ps4Joystick.povRight(),
  //                                                                SubsystemManager.ps4Joystick.povLeft());

  // // Applies deadbands and inverts controls because joysticksy
  // // are back-right positive while robot
  // // controls are front-left positive
  // // left stick controls translation
  // // right stick controls the desired angle NOT angular rotation
  // Command driveFieldOrientedDirectAngle = SubsystemManager.getDriveBase().driveCommand(
  //     () -> MathUtil.applyDeadband(-SubsystemManager.getpsJoystick().getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  //     () -> MathUtil.applyDeadband(-SubsystemManager.getpsJoystick().getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
  //     () -> -SubsystemManager.getpsJoystick().getRightX(),
  //     () -> -SubsystemManager.getpsJoystick().getRightY());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot

  Command driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
      () -> MathUtil.applyDeadband(-SubsystemManager.getpsJoystick().getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-SubsystemManager.getpsJoystick().getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -SubsystemManager.getpsJoystick().getRightX());
      
  SlewRateLimiter joystickSlewRateLimiter = new SlewRateLimiter(4);

  public RobotContainer()
  {
    // Configure the trigger bindings
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
    // NamedCommands.registerCommand("Align", SubsystemManager.alignCommand);
    // NamedCommands.registerCommand("SelectLeftLimelight", SubsystemManager.selectLeftLimelight);
    // NamedCommands.registerCommand("SelectRightLimelight", SubsystemManager.selectRightLimelight);
    // NamedCommands.registerCommand("SelectMidLimelight", SubsystemManager.selectMidLimelight);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {    
    SubsystemManager.getpsJoystick().PS().onTrue((Commands.runOnce(SubsystemManager.getDrivebase()::zeroGyroWithAlliance)));
      // we will use it in case our rotation is be messed up
  }

  private void configureDriveCommand() {
    if (teamColorIsBlue()) {
      SubsystemManager.getDrivebase().zeroGyro();
      SubsystemManager.getDrivebase().resetOdometry(new Pose2d(SubsystemManager.getDrivebase().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(-SubsystemManager.getpsJoystick().getLeftY())),
        () -> (modifyAxis(-SubsystemManager.getpsJoystick().getLeftX())),
        () -> (modifyAxis(-SubsystemManager.getpsJoystick().getRightX())));
    } else {
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand( // default
        () -> (modifyAxis(SubsystemManager.getpsJoystick().getLeftY())),
        () -> (modifyAxis(SubsystemManager.getpsJoystick().getLeftX())),
        () -> (modifyAxis(-SubsystemManager.getpsJoystick().getRightX())));
    }
    
    SubsystemManager.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return SubsystemManager.getDrivebase().getAutonomousCommand(Dashboard.getSelectedAutonomy());
  }

  public void setDriveMode()
  {}

  public void setMotorBrake(boolean brake)
  {
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

  private static double modifyAxis(double joystickInput) {
    double deadBandValue = MathUtil.applyDeadband(joystickInput, OperatorConstants.LEFT_Y_DEADBAND);
    double scaled_input = 1 - (1 - Math.abs(deadBandValue)) * (1 - 0.1);
    return deadBandValue * scaled_input;
  }
}

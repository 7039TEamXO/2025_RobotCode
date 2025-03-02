// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SubsystemManager;

import java.util.Map;
import java.util.Optional;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // final CommandPS4Controller SubsystemManager.ps4Joystick = new CommandPS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

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

  Command driveFieldOrientedAngularVelocity = SubsystemManager.getDriveBase().driveCommand( // default
      () -> MathUtil.applyDeadband(-SubsystemManager.getpsJoystick().getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-SubsystemManager.getpsJoystick().getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -SubsystemManager.getpsJoystick().getRightX());
      
  // Command alignRightCommand =  SubsystemManager.getDriveBase().alignByLimelight(modifyAxis(-SubsystemManager.getpsJoystick().getLeftY()));

  SlewRateLimiter joystickSlewRateLimiter = new SlewRateLimiter(4);
  // Command driveFieldOrientedDirectAngleSim = SubsystemManager.getDriveBase().simDriveCommand(
  //     () -> MathUtil.applyDeadband(SubsystemManager.ps4Joystick.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  //     () -> MathUtil.applyDeadband(SubsystemManager.ps4Joystick.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
  //     () -> SubsystemManager.ps4Joystick.getRawAxis(2));

  // drivebase.setDefaultCommand(
  //     false ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  
  // SubsystemManager.ps4Joystick.povUp().toggleOnTrue(closedAbsoluteDriveAdv);
  // SubsystemManager.ps4Joystick.povLeft().toggleOnTrue(closedAbsoluteDriveAdv);
  // SubsystemManager.ps4Joystick.povRight().toggleOnTrue(closedAbsoluteDriveAdv);
  // SubsystemManager.ps4Joystick.povDown().toggleOnTrue(closedAbsoluteDriveAdv);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    SubsystemManager.getpsJoystick().PS().onTrue((Commands.runOnce(SubsystemManager.getDriveBase()::zeroGyroWithAlliance)));// if we will use it, in case if driver push this buttom, our rotation will be messed up

    // chnage pipeling when we press align
    SubsystemManager.getpsJoystick().R1().onTrue(Commands.runOnce( () -> Limelight.setPipeline(0)));
    SubsystemManager.getpsJoystick().L1().onTrue(Commands.runOnce( () -> Limelight.setPipeline(1)));
    SubsystemManager.getpsJoystick().button(12).onTrue(Commands.runOnce( () -> Limelight.setPipeline(2)));

    // Set priority tag when we press align, reset on release    
    SubsystemManager.getpsJoystick().button(12).onTrue(Commands.runOnce( () -> Limelight.setPriorityTagId(Limelight.getMainAprilTagId())));
    SubsystemManager.getpsJoystick().button(12).onFalse(Commands.runOnce( () -> Limelight.resetPriorityTagId()));
    SubsystemManager.getpsJoystick().R1().onTrue(Commands.runOnce( () -> Limelight.setPriorityTagId(Limelight.getMainAprilTagId())));
    SubsystemManager.getpsJoystick().R1().onFalse(Commands.runOnce( () -> Limelight.resetPriorityTagId()));
    SubsystemManager.getpsJoystick().L1().onTrue(Commands.runOnce( () -> Limelight.setPriorityTagId(Limelight.getMainAprilTagId())));
    SubsystemManager.getpsJoystick().L1().onFalse(Commands.runOnce( () -> Limelight.resetPriorityTagId()));

    //======================//

    // Align only by TX and not reef angle 
    // SubsystemManager.getpsJoystick().R1().whileTrue(SubsystemManager.getDriveBase().alignByLimelight( () -> modifyAxis(-SubsystemManager.getpsJoystick().getLeftY())));
    // SubsystemManager.getpsJoystick().L1().whileTrue(SubsystemManager.getDriveBase().alignByLimelight( () -> modifyAxis(-SubsystemManager.getpsJoystick().getLeftY())));
    // SubsystemManager.getpsJoystick().button(12).whileTrue(SubsystemManager.getDriveBase().alignByLimelight( () -> modifyAxis(-SubsystemManager.getpsJoystick().getLeftY())));
    
    SubsystemManager.getpsJoystick().button(12).whileTrue(SubsystemManager.getDriveBase().advancedAlignByLimelight( () -> modifyAxis(-SubsystemManager.getpsJoystick().getLeftY()), Limelight.getMainAprilTagId()));
    SubsystemManager.getpsJoystick().R1().whileTrue(SubsystemManager.getDriveBase().advancedAlignByLimelight( () -> modifyAxis(-SubsystemManager.getpsJoystick().getLeftY()), Limelight.getMainAprilTagId()));
    SubsystemManager.getpsJoystick().L1().whileTrue(SubsystemManager.getDriveBase().advancedAlignByLimelight( () -> modifyAxis(-SubsystemManager.getpsJoystick().getLeftY()), Limelight.getMainAprilTagId()));

  }


  private void configureDriveCommand() {
    if (teamColorIsBlue()) {
      // SubsystemManager.getDriveBase().zeroGyroWithAlliance();
      SubsystemManager.getDriveBase().zeroGyro();
      SubsystemManager.getDriveBase().resetOdometry(new Pose2d(SubsystemManager.getDriveBase().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      driveFieldOrientedAngularVelocity = SubsystemManager.getDriveBase().driveCommand( // default
      () -> (modifyAxis(-SubsystemManager.getpsJoystick().getLeftY())),
      () -> (modifyAxis(-SubsystemManager.getpsJoystick().getLeftX())),
      () -> (modifyAxis(-SubsystemManager.getpsJoystick().getRightX())));
    } else {
      // SubsystemManager.getDriveBase().zeroGyroWithAlliance();
      driveFieldOrientedAngularVelocity = SubsystemManager.getDriveBase().driveCommand( // default
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
    return SubsystemManager.getDriveBase().getAutonomousCommand(Dashboard.getSelectedAutonomy());
  }

  public void setDriveMode()
  {
  }

  public void setMotorBrake(boolean brake)
  {
    SubsystemManager.getDriveBase().setMotorBrake(brake);
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

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueLeftCoralAuto extends SequentialCommandGroup {
    public BlueLeftCoralAuto() {
        addCommands(new RedRightCoralAuto(
            new Pose2d(17.5 - 10.135, 8 - 6, new Rotation2d(Math.toRadians(180))),
            new Pose2d(17.5 - 12.53, 8 - 5.27, new Rotation2d(Math.toRadians(180 - 60))),
            new Pose2d(17.5 - 13.59, 8 - 5.27, new Rotation2d(Math.toRadians(180 - 120))),
            new Pose2d(17.5 - 13.87, 8 - 5.10, new Rotation2d(Math.toRadians(180 - 120))),
            new Pose2d(17.5 - 16.20, 8 - 7.5, new Rotation2d(Math.toRadians(180 - 180)))
        ));
    }
}

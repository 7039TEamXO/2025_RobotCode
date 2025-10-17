package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedLeftCoralAuto extends SequentialCommandGroup {
    public RedLeftCoralAuto() {
        addCommands(new RedRightCoralAuto(
            new Pose2d(10.135, 8 - 6, new Rotation2d()),
            new Pose2d(12.53, 8 - 5.27, new Rotation2d(Math.toRadians(60))),
            new Pose2d(13.59, 8 - 5.27, new Rotation2d(Math.toRadians(120))),
            new Pose2d(13.87, 8 - 5.10, new Rotation2d(Math.toRadians(120))),
            new Pose2d(16.20, 8 - 7.5, new Rotation2d(Math.toRadians(180)))
        ));
    }
}

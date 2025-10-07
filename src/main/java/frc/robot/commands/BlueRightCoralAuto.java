package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueRightCoralAuto extends SequentialCommandGroup {
    public BlueRightCoralAuto() {
        addCommands(new RedRightCoralAuto(
            new Pose2d(17.5 - 10.135, 6, new Rotation2d(Math.toRadians(180))),
            new Pose2d(17.5 - 12.53, 5.27, new Rotation2d(Math.toRadians(180 + 60))),
            new Pose2d(17.5 - 13.59, 5.27, new Rotation2d(Math.toRadians(180 + 120))),
            new Pose2d(17.5 - 13.87, 5.10, new Rotation2d(Math.toRadians(180 + 120))),
            new Pose2d(17.5 - 14.40, 4.19, new Rotation2d(Math.toRadians(180 + 180)))
        ));
    }
}

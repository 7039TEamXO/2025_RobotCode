package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.IO.CameraIO;
import frc.robot.subsystems.IO.CameraIO.CameraIOInputs;

public class Limelight {
    private static CameraIO io;
    private static CameraIOInputs inputs = new CameraIOInputs();

    public static void init(CameraIO _io) {
        io = _io;

        io.init();
        io.updateInputs(inputs);
        Logger.processInputs("Limelight", inputs);
    }

    public static Object[] update() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Limelight", inputs);

        if(!inputs.doRejectUpdate) {
            return new Object[]{ inputs.estimatedPose, inputs.timestampSeconds, VecBuilder.fill(inputs.stdDevs[0], inputs.stdDevs[1], inputs.stdDevs[2]) };
        }
        
        return null;  
    }

    public static boolean hasTarget() {
        return inputs.hasTarget;
    }

    public static int getMainAprilTagID() {
        return inputs.mainAprilTagID;
    }

    public static double getTX() {
        return inputs.TX;
    }

    public static double getTA() {
        return inputs.TA;
    }

    public static boolean filterTargetByTA() {
        return getTA() > 0.15 && hasTarget();
    }
}
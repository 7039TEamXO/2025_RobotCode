package frc.robot.subsystems.Handler;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class HandlerTuning {
    private static final LoggedNetworkNumber CORAL_IN_DEBOUNCE_COUNTER = new LoggedNetworkNumber("/Tuning/Handler/Coral Bounce Counter", HandlerConstants.CORAL_IN_DEBOUNCE_COUNTER);
    private static final LoggedNetworkNumber LEVEL4_CORAL_PUSH_DISTANCE = new LoggedNetworkNumber("/Tuning/Handler/Level 4 Coral Push Distance", HandlerConstants.LEVEL4_CORAL_PUSH_DISTANCE);
    private static final LoggedNetworkNumber ALGAE_PROC_IR_IN_VALUE = new LoggedNetworkNumber("/Tuning/Handler/Algae Processor In Value", HandlerConstants.ALGAE_PROC_IR_IN_VALUE);
    private static final LoggedNetworkNumber ALGAE_NET_IR_IN_VALUE = new LoggedNetworkNumber("/Tuning/Handler/Algae Net In Value", HandlerConstants.ALGAE_NET_IR_IN_VALUE);

    public static final double CORAL_IN_DEBOUNCE_COUNTER_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CORAL_IN_DEBOUNCE_COUNTER.get() : HandlerConstants.CORAL_IN_DEBOUNCE_COUNTER;
    }

    public static final double LEVEL4_CORAL_PUSH_DISTANCE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? LEVEL4_CORAL_PUSH_DISTANCE.get() : HandlerConstants.LEVEL4_CORAL_PUSH_DISTANCE;
    }

    public static final double ALGAE_PROC_IR_IN_VALUE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? ALGAE_PROC_IR_IN_VALUE.get() : HandlerConstants.ALGAE_PROC_IR_IN_VALUE;
    }

    public static final double ALGAE_NET_IR_IN_VALUE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? ALGAE_NET_IR_IN_VALUE.get() : HandlerConstants.ALGAE_NET_IR_IN_VALUE;
    }
}

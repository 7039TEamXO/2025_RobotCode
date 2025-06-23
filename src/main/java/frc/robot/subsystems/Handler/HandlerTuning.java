package frc.robot.subsystems.Handler;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class HandlerTuning {
    private static final LoggedNetworkNumber CORAL_IN_DEBOUNCE_COUNTER = new LoggedNetworkNumber("/Tuning/Handler/CORAL_IN_DEBOUNCE_COUNTER", HandlerConstants.CORAL_IN_DEBOUNCE_COUNTER);
    private static final LoggedNetworkNumber LEVEL4_CORAL_PUSH_DISTANCE = new LoggedNetworkNumber("/Tuning/Handler/LEVEL4_CORAL_PUSH_DISTANCE", HandlerConstants.LEVEL4_CORAL_PUSH_DISTANCE);
    private static final LoggedNetworkNumber ALGAE_PROC_IR_IN_VALUE = new LoggedNetworkNumber("/Tuning/Handler/ALGAE_PROC_IR_IN_VALUE", HandlerConstants.ALGAE_PROC_IR_IN_VALUE);
    private static final LoggedNetworkNumber ALGAE_NET_IR_IN_VALUE = new LoggedNetworkNumber("/Tuning/Handler/ALGAE_NET_IR_IN_VALUE", HandlerConstants.ALGAE_NET_IR_IN_VALUE);

    public static final double CORAL_IN_DEBOUNCE_COUNTER_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? CORAL_IN_DEBOUNCE_COUNTER.get() : HandlerConstants.CORAL_IN_DEBOUNCE_COUNTER;
    }

    public static final double LEVEL4_CORAL_PUSH_DISTANCE_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? LEVEL4_CORAL_PUSH_DISTANCE.get() : HandlerConstants.LEVEL4_CORAL_PUSH_DISTANCE;
    }

    public static final double ALGAE_PROC_IR_IN_VALUE_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? ALGAE_PROC_IR_IN_VALUE.get() : HandlerConstants.ALGAE_PROC_IR_IN_VALUE;
    }

    public static final double ALGAE_NET_IR_IN_VALUE_get() {
        return Constants.CurrentTuningMode == TuningMode.TUNE ? ALGAE_NET_IR_IN_VALUE.get() : HandlerConstants.ALGAE_NET_IR_IN_VALUE;
    }
}

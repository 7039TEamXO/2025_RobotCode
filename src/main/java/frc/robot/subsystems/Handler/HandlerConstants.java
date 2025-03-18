package frc.robot.subsystems.Handler;

public class HandlerConstants {
    public static final int HandlerMotorID = 1;

    public static final int HandlerAnalogProcInputSensorID = 3;
    public static final int HandlerAnalogNetInputSensorID = 1;
    public static final int HandlerDigitalInputSensorID = 1;
    public static final int HandlerDigitalOutputSensorID = 0;

    public static final double HANDLER_POWER_INTAKE_ALGAE = 0.55; // 0.55
    public static final double HANDLER_POWER_DEPLETE_CORAL = 0.55;
    public static final double HANDLER_POWER_STOP = 0;
    public static final double HANDLER_POWER_DEPLETE_ALGAE = -0.6;// -0.6
    public static final double HANDLER_POWER_HOLD_ALGAE = 0.3;// 0.3
    public static final double HANDLER_POWER_INTAKE_CORAL_TELEOP = 0.2;
    
    public static final double HANDLER_POWER_INTAKE_CORAL_AUTO = 0.35;//0.3
    public static final double HANDLER_POWER_DEPLETE_CORAL_LEVEL0 = 0.3;
    public static final double HANDLER_POWER_FEED_CORAL = -0.05;
    public static final double HANDLER_POWER_PUSH_BACK_CORAL = -0.05;

    public static final double HANDLER_POWER_HOLD_NET = -0.15;
    public static final double HANDLER_POWER_INTAKE_NET = -0.55;
    public static final double HANDLER_POWER_DEPLETE_NET = 1;

    public static final int CORAL_IN_DEBOUNCE_COUNTER = 16;
    public static final double LEVEL4_CORAL_PUSH_DISTANCE = 1.5;

    public static final double ALGAE_PROC_IR_IN_VALUE = 237;
    public static final double ALGAE_NET_IR_IN_VALUE = -1; // CHANGE [!]

    public static final double ALGAE_IN_CURRENT = 50; // REMOVE [!]
}

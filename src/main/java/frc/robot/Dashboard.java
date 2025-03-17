package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DeliveryManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.Handler.*;
import frc.robot.subsystems.Tray.Tray;

public class Dashboard {
    private final static SendableChooser<String> autoChooser = new SendableChooser<>();

    private final static SendableChooser<String> chosenElevatorStates = new SendableChooser<>();
    private final static SendableChooser<String> chosenWristStates = new SendableChooser<>();
    private final static SendableChooser<String> chosenHandlerStates = new SendableChooser<>();

    private static HttpCamera limelightCamera = new HttpCamera("limelight", "http://10.70.39.203:5801");
    private static HttpCamera limelightClimbCamera = new HttpCamera("limelight-camera", "http://10.70.39.200:5801/");

    private static NetworkTableEntry acceptStateChanges;
    private static NetworkTableEntry acceptCoralChanges;
    private static NetworkTableEntry chosenIsCoralIn;
    private static NetworkTableEntry acceptAlgaeChanges;
    private static NetworkTableEntry chosenIsAlgaeInProcessor;
    private static NetworkTableEntry chosenIsAlgaeInNet;

    private static NetworkTableEntry addValueToWrist;
    private static NetworkTableEntry addValueToElevator;
    private static NetworkTableEntry addValueToHandler;

    public static void init() {
        Autos[] states = Autos.values();
        for (int i = 0; i < states.length; i++) {
            Autos state = states[i];
            String name_state = state.getAutoName();
            if (i == 0) {
                autoChooser.setDefaultOption(name_state, name_state);
            }
            autoChooser.addOption(name_state, name_state);
        }
    
        // -----------------------
          
        SmartDashboard.putData("Autos", autoChooser);
        
        SmartDashboard.putBoolean("Accept State Changes", false);
        acceptStateChanges = SmartDashboard.getEntry("Accept State Changes");

        SmartDashboard.putBoolean("Accept Coral Changes", false);
        acceptCoralChanges = SmartDashboard.getEntry("Accept Coral Changes");

        SmartDashboard.putBoolean("Choose isCoralIn", false);
        chosenIsCoralIn = SmartDashboard.getEntry("Choose isCoralIn");

        SmartDashboard.putBoolean("Accept Algae Changes", false);
        acceptAlgaeChanges = SmartDashboard.getEntry("Accept Algae Changes");

        SmartDashboard.putBoolean("Choose isAlgaeIn (Processor)", false);
        chosenIsAlgaeInProcessor = SmartDashboard.getEntry("Choose isAlgaeIn (Processor)");

        SmartDashboard.putBoolean("Choose isAlgaeIn (Net)", false);
        chosenIsAlgaeInNet = SmartDashboard.getEntry("Choose isAlgaeIn (Net)");

        SmartDashboard.putNumber("Add Elevator Value", 0);
        addValueToElevator = SmartDashboard.getEntry("Add Elevator Value");

        SmartDashboard.putNumber("Add Wrist Value", 0);
        addValueToWrist = SmartDashboard.getEntry("Add Wrist Value");

        SmartDashboard.putNumber("Add Handler Value", 0);
        addValueToHandler = SmartDashboard.getEntry("Add Handler Value");

        // SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("HandlerEncoder", () -> Handler.getHandlerMotorDistance());
        
        // debugging.addNumber("HandlerCounter", () -> Handler.getCoralIntakeCounter());
        // debugging.addBoolean("HandlerReset", () -> Handler.getReset());

        // debugging.addString("ElevatorState",() -> SubsystemManager.getElevatorState().name());
        // debugging.addString("LastElevatorState",() -> SubsystemManager.getLastElevatorState().name());        

        update();

        // --------
        setElevatorState();
        setWristState();
        setHandlerState();
    }

    public static void update() {
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("X", SubsystemManager.getDriveBase().getPose().getX()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Y", SubsystemManager.getDriveBase().getPose().getY()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Rotation", SubsystemManager.getDriveBase().getPose().getRotation().getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Front Left", SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[0].getPosition().angle.getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Front Right", SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[1].getPosition().angle.getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Back Left", SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[2].getPosition().angle.getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Back Right", SubsystemManager.getDriveBase().getSwerveDriveConfiguration().modules[3].getPosition().angle.getDegrees()));

        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Elevator Raw", Elevator.getCurrentPosition()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Wrist Raw", Wrist.getCurrentPosition()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Tray Raw", Tray.getTrayPosition()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Climb Raw", Climb.getClimbPose()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putBoolean("Coral IR", Handler.getCoralIr()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Algae IR (Processor)", Handler.getAlgaeProcIrValue()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Algae IR (Net)", Handler.getAlgaeNetIrValue()));

        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Elevator State", SubsystemManager.getElevatorState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Robot State", SubsystemManager.getRobotState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Handler State", SubsystemManager.getHandlerState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Wrist State", DeliveryManager.getWristState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Climb State", SubsystemManager.getClimbState().name()));

        SmartDashboard.postListenerTask(() -> SmartDashboard.putBoolean("Is Coral In", Handler.isCoralIn()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putBoolean("Is Algae In (Processor)", Handler.isAlgaeInProcessor()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putBoolean("Is Algae In (Net)", Handler.isAlgaeInNet()));

        SmartDashboard.updateValues();
    }

    public static boolean getAcceptCoralChanges() {
        return acceptCoralChanges.getBoolean(false);
    }

    public static boolean getChosenIsCoralIn() {
        return chosenIsCoralIn.getBoolean(false);
    }

    public static boolean getAcceptAlgaeChanges() {
        return acceptAlgaeChanges.getBoolean(false);
    }

    public static boolean getChosenIsAlgaeInProcessor() {
        return chosenIsAlgaeInProcessor.getBoolean(false);
    }

    public static boolean getChosenIsAlgaeInNet() {
        return chosenIsAlgaeInNet.getBoolean(false);
    }

    public static boolean getAcceptChanges() {
        return acceptStateChanges.getBoolean(false);
    }

    public static void setElevatorState() {
        ElevatorState[] elevatorStates = ElevatorState.values();
        for (int k = 0; k < elevatorStates.length; k++) {
            ElevatorState elevatorState = elevatorStates[k];
            if (k == 0) {
                chosenElevatorStates.setDefaultOption(elevatorState.name(), elevatorState.name());
            }
            chosenElevatorStates.addOption(elevatorState.name(), elevatorState.name());
        }
        SmartDashboard.putData("Choose Elevator State", chosenElevatorStates);
    }

    public static ElevatorState getSelectedElevatorState() {
        return ElevatorState.valueOf(chosenElevatorStates.getSelected());
    }

    public static void setWristState() {
        WristState[] wristStates = WristState.values();
        for (int i = 0; i < wristStates.length; i++) {
            WristState wristState = wristStates[i];
            if (i == 0) {
                chosenWristStates.setDefaultOption(wristState.name(), wristState.name());
            }
            chosenWristStates.addOption(wristState.name(), wristState.name());
        }
        SmartDashboard.putData("Choose Wrist State", chosenWristStates);
    }
    
    public static WristState getSelectedWristState() {
        return WristState.valueOf(chosenWristStates.getSelected()); 
    }

    public static void setHandlerState() {
        HandlerState[] handlerStates = HandlerState.values();
        for (int i = 0; i < handlerStates.length; i++) {
            HandlerState handlerStaste = handlerStates[i];
            if (i == 0) {
                chosenHandlerStates.setDefaultOption(handlerStaste.name(), handlerStaste.name());
            }
            chosenHandlerStates.addOption(handlerStaste.name(), handlerStaste.name());
        }
        SmartDashboard.putData("Choose Handler State", chosenHandlerStates);
    }
    
    public static HandlerState getSelectedHandlerState() {
        return HandlerState.valueOf(chosenHandlerStates.getSelected()); 
    }
        
    public static String getSelectedAutonomy() {
        return autoChooser.getSelected();
    }

    public static double addValueToElevator() {
        return addValueToElevator.getDouble(0);
    }

    public static double addValueToWrist() {
        return addValueToWrist.getDouble(0);
    }

    public static double addValueToHandler() {
        return addValueToHandler.getDouble(0);
    }

    // public static void cameraInit() {
    //     UsbCamera usbCamera = new UsbCamera("USB Camera 0",0);
    //     MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    //     mjpegServer1.setSource(usbCamera);
    //     // Creates the CvSink and connects it to the UsbCamera
    //     CvSink cvSink = new CvSink("opencv_USB Camera 0");
    //     cvSink.setSource(usbCamera);
    //     // Creates the CvSource and MjpegServer [2] and connects them
    //     CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    //     MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    //     mjpegServer2.setSource(outputStream);
    // }
}
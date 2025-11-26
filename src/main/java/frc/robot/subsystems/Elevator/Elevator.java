package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Dashboard;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.IO.ElevatorIO;
import frc.robot.subsystems.IO.ElevatorIO.ElevatorIOInputs;

public class Elevator implements Subsystem {
    private double elevatorPosition; 
    private final MotionMagicExpoVoltage motorRequest = new MotionMagicExpoVoltage(0);

    private ElevatorIO io;
    private ElevatorIOInputs inputs = new ElevatorIOInputs();

    public Elevator(ElevatorIO _io) {
        io = _io;

        io.setLeadMotorPosition(0);
        io.setFollowerMotorPosition(0);

        setMotorConfigs();

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ElevatorConstants.SysIdQuasistatic), 
                Volts.of(ElevatorConstants.SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> io.setVoltage(v.plus(Volts.of(ElevatorConstants.kG))), null, this));
    }

    public void operate(ElevatorState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        switch (state) {
            case BASE: 
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_BASE;
                break;

            case ALGAE_HIGH_PROCESSOR:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_HIGH;
                break;

            case ALGAE_LOW_PROCESSOR:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_LOW;
                break;

            case LEVEL0:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL0;
                break;

            case LEVEL1:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL1;
                break;

            case LEVEL2:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL2;
                break;

            case ALGAE_HOLD_NET:
            case LEVEL3:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_LEVEL3;
                break;

            case INTAKE_CORAL:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_INTAKE_CORAL;
                break;

            case ALGAE_HIGH_IN:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_HIGH + ElevatorConstants.ELEVATOR_POSE_ALGAE_IN_OFFSET;
                break;

            case ALGAE_LOW_IN:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_LOW + ElevatorConstants.ELEVATOR_POSE_ALGAE_IN_OFFSET;
                break;
                
            case ALGAE_HIGH_NET:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_HIGH_NET;
                break;

            case ALGAE_LOW_NET:
                elevatorPosition = ElevatorConstants.ELEVATOR_POSE_ALGAE_LOW_NET;
                break;
        }

        elevatorPosition = elevatorPosition + Dashboard.addValueToElevator();
        io.setLeadMotionMagic(motorRequest.withPosition(elevatorPosition));
    }

    public double getCurrentPosition() {
        return inputs.masterPosition;
    }

    public double getCurrentVelocity() {
        return inputs.masterVelocity;
    }

    public double getCurrentVoltage() {
        return inputs.masterAppliedVolts;
    }

    private void setMotorConfigs() {
        io.setFollowerMotionMagic(io.createFollower());

        var rightTalonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = rightTalonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorTuning.kS_get();
        slot0Configs.kV = ElevatorTuning.kV_get();
        slot0Configs.kA = ElevatorTuning.kA_get();
        slot0Configs.kP = ElevatorTuning.kP_get();
        slot0Configs.kI = ElevatorTuning.kI_get();
        slot0Configs.kD = ElevatorTuning.kD_get();

        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kG = ElevatorTuning.kG_get();

        var motionMagicConfigs = rightTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorTuning.MMExpo_CV_get();
        motionMagicConfigs.MotionMagicExpo_kV = ElevatorTuning.MMExpo_kV_get();
        motionMagicConfigs.MotionMagicExpo_kA = ElevatorTuning.MMExpo_kA_get();
        
        rightTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightTalonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
        rightTalonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightTalonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SupplyCurrentLimit;
        rightTalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightTalonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1;
        rightTalonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.6;

        io.applyMasterTalonFXConfig(rightTalonFXConfigs);

        var leftTalonFXConfigs = new TalonFXConfiguration();

        leftTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorCurrentLimit;
        leftTalonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftTalonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SupplyCurrentLimit;
        leftTalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftTalonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1;
        leftTalonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.6;

        io.applySlaveTalonFXConfig(leftTalonFXConfigs);
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
    
    // SysId routines //

    private SysIdRoutine sysIdRoutine;    

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
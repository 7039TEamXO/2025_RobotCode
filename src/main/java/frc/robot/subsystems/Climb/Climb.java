package frc.robot.subsystems.Climb;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.IO.ClimbIO;
import frc.robot.subsystems.IO.ClimbIO.ClimbIOInputs;

public class Climb implements Subsystem {
    private ClimbIO io;
    private ClimbIOInputs inputs = new ClimbIOInputs();

    private double wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
    // private double wantedPose = 0;

    public Climb(ClimbIO _io) {
        io = _io;

        setMotorConfigs();
        io.setPosition(0);

        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        // sysIdRoutine = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         Volts.per(Second).of(ClimbConstants.SysIdQuasistatic), 
        //         Volts.of(ClimbConstants.SysIdDynamic), null, state -> {
        //              SignalLogger.writeString("state", state.toString());
        //     }),
        //     new SysIdRoutine.Mechanism(v -> io.setVoltage(v), null, this));
    }

    public void operate(ClimbState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        switch (state) {
            case STOP:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
                break;
            
            case OPEN:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_STOP;
                break;
            
            case CLIMB:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_CLIMB;
                break;
            
            case DESCEND:
                wantedPower = ClimbConstants.CLIMB_WANTED_POWER_DESCEND;
                break;

            case TRAVEL:
                // wantedPose = ClimbTuning.CLIMB_TRAVEL_POSE_get();
                break;
        }
        
        io.setMotionMagic(new DutyCycleOut(wantedPower));
    } 

    private void setMotorConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ClimbTuning.kS_get(); // Add 0.25 V output to overcome friction
        slot0Configs.kV = ClimbTuning.kV_get(); // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ClimbTuning.kA_get(); // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ClimbTuning.kP_get(); // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ClimbTuning.kI_get(); // no output for integrated error
        slot0Configs.kD = ClimbTuning.kD_get(); // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbTuning.MotionMagicCruiseVelocity_get(); // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = ClimbTuning.MotionMagicAcceleration_get(); // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = ClimbTuning.MotionMagicJerk_get(); // Target jerk of 1600 rps/s/s (0.1 seconds)
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ClimbConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyTalonFXConfig(talonFXConfigs);
    }

    public double getCurrentPosition() {
        return inputs.position;
    }

    public double getCurrentVelocity() {
        return inputs.velocity;
    }

    public double getCurrentVoltage() {
        return inputs.appliedVolts;
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    // SysId routines //

    // private SysIdRoutine sysIdRoutine;    

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.dynamic(direction);
    // }
}

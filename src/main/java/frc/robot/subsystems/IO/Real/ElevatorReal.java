package frc.robot.subsystems.IO.Real;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.IO.ElevatorIO;

public class ElevatorReal implements ElevatorIO {
    private final TalonFX elevatorMasterMotor;
    private final TalonFX elevatorSlaveMotor;
    
    public ElevatorReal() {
        elevatorMasterMotor = new TalonFX(ElevatorConstants.ElevatorRightMotorID);
        elevatorSlaveMotor = new TalonFX(ElevatorConstants.ElevatorLeftMotorID);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.masterPosition = elevatorMasterMotor.getPosition().getValueAsDouble();
        inputs.masterVelocity = elevatorMasterMotor.getVelocity().getValueAsDouble();
        inputs.masterAppliedVolts = elevatorMasterMotor.getMotorVoltage().getValueAsDouble();
        inputs.masterCurrentAmps = elevatorMasterMotor.getStatorCurrent().getValueAsDouble();

        inputs.slavePosition = elevatorSlaveMotor.getPosition().getValueAsDouble();
        inputs.slaveVelocity = elevatorSlaveMotor.getVelocity().getValueAsDouble();
        inputs.slaveAppliedVolts = elevatorSlaveMotor.getMotorVoltage().getValueAsDouble();
        inputs.slaveCurrentAmps = elevatorSlaveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration) {
        elevatorMasterMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration) {
        elevatorSlaveMotor.getConfigurator().apply(configuration);
    }

    @Override
    public Follower createFollower() {
        return new Follower(elevatorMasterMotor.getDeviceID(), true);
    }

    @Override
    public void setLeadMotorPosition(double newValue) {
        elevatorMasterMotor.setPosition(newValue);
    }

    @Override
    public void setFollowerMotorPosition(double newValue) {
        elevatorSlaveMotor.setPosition(newValue);
    }

    @Override
    public void setLeadMotionMagic(MotionMagicExpoVoltage request) {
        elevatorMasterMotor.setControl(request);
    }

    @Override
    public void setFollowerMotionMagic(Follower request) {
        elevatorSlaveMotor.setControl(request);
    }

    @Override
    public void setVoltage(Voltage volts) {
        elevatorMasterMotor.setVoltage(volts.in(Volts));
        elevatorSlaveMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void simulationPeriodic() {}
    
    @Override
    public void close() throws Exception {
        elevatorMasterMotor.close();
        elevatorSlaveMotor.close();
    }
}

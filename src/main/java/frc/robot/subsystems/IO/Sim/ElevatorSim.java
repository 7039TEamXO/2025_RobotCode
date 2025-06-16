package frc.robot.subsystems.IO.Sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.IO.ElevatorIO;

public class ElevatorSim implements ElevatorIO {
    private final TalonFX masterMotor;
    private final TalonFX slaveMotor;
    private final TalonFXSimState masterMotorSim;
    private final TalonFXSimState slaveMotorSim;

    private final edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSimModel;

    private final double periodicDt;

    private class ElevatorSimConstants {
        private static final double GearRatio = 10.0;
        private static final double DrumRadius = 0.07;
        private static final double ElevatorMass = 10.0;
        private static final double MetersPerRotation = DrumRadius * 2 * Math.PI / GearRatio;
        private static final double MaxHeight = 2 * ElevatorConstants.EncoderMaxPos * MetersPerRotation;
    }

    public ElevatorSim(double _periodicDt) {
        periodicDt = _periodicDt;

        masterMotor = new TalonFX(ElevatorConstants.ElevatorRightMotorID);
        slaveMotor = new TalonFX(ElevatorConstants.ElevatorLeftMotorID);

        masterMotorSim = masterMotor.getSimState();
        slaveMotorSim = slaveMotor.getSimState();

        elevatorSimModel =  new edu.wpi.first.wpilibj.simulation.ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorSimConstants.GearRatio,
            ElevatorSimConstants.ElevatorMass,
            ElevatorSimConstants.DrumRadius,
            0,
            ElevatorSimConstants.MaxHeight,
            true,
            0,
            0.0003,
            0.0003
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.masterPosition = masterMotor.getPosition().getValueAsDouble();
        inputs.masterVelocity = masterMotor.getVelocity().getValueAsDouble();
        inputs.masterAppliedVolts = masterMotor.getMotorVoltage().getValueAsDouble();
        inputs.masterCurrentAmps = masterMotor.getStatorCurrent().getValueAsDouble();

        inputs.slavePosition = slaveMotor.getPosition().getValueAsDouble();
        inputs.slaveVelocity = slaveMotor.getVelocity().getValueAsDouble();
        inputs.slaveAppliedVolts = slaveMotor.getMotorVoltage().getValueAsDouble();
        inputs.slaveCurrentAmps = slaveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration) {
        masterMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration) {
        slaveMotor.getConfigurator().apply(configuration);
    }

    @Override
    public Follower createFollower() {
        return new Follower(masterMotor.getDeviceID(), false);
    }

    @Override
    public void setLeadMotorPosition(double newValue) {
        masterMotor.setPosition(newValue);
    }

    @Override
    public void setFollowerMotorPosition(double newValue) {
        slaveMotor.setPosition(newValue);
    }

    @Override
    public void setLeadMotionMagic(MotionMagicVoltage request) {
        masterMotor.setControl(request);
    }

    @Override
    public void setFollowerMotionMagic(Follower request) {
        slaveMotor.setControl(request);
    }

    @Override
    public void simulationPeriodic() {
        masterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        slaveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var masterMotorVoltage = masterMotorSim.getMotorVoltage();
        var slaveMotorVoltage = slaveMotorSim.getMotorVoltage();

        elevatorSimModel.setInputVoltage((masterMotorVoltage + slaveMotorVoltage) / 2.0);
        elevatorSimModel.update(periodicDt);
    
        masterMotorSim.setRawRotorPosition(
            elevatorSimModel.getPositionMeters() / ElevatorSimConstants.MetersPerRotation);
        masterMotorSim.setRotorVelocity(
            elevatorSimModel.getVelocityMetersPerSecond() / ElevatorSimConstants.MetersPerRotation);
        slaveMotorSim.setRawRotorPosition(
            elevatorSimModel.getPositionMeters() / ElevatorSimConstants.MetersPerRotation);
        slaveMotorSim.setRotorVelocity(
            elevatorSimModel.getVelocityMetersPerSecond() / ElevatorSimConstants.MetersPerRotation);
    }
    
    @Override
    public void close() throws Exception {
        masterMotor.close();
        slaveMotor.close();
    }
}

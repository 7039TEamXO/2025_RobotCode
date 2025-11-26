package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.IO.ElevatorIO;

public class ElevatorStub implements ElevatorIO {
    public ElevatorStub() {}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {}

    @Override
    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public Follower createFollower() {
        return new Follower(0, false);
    }

    @Override
    public void setLeadMotorPosition(double newValue) {}

    @Override
    public void setFollowerMotorPosition(double newValue) {}

    @Override
    public void setLeadMotionMagic(MotionMagicExpoVoltage request) {}

    @Override
    public void setFollowerMotionMagic(Follower request) {}

    @Override
    public void setVoltage(Voltage voltage) {}

    @Override
    public void simulationPeriodic() {}
    
    @Override
    public void close() throws Exception {}
}

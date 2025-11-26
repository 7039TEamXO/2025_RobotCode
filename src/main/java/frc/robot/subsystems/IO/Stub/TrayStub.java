package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.IO.TrayIO;

public class TrayStub implements TrayIO {
    public TrayStub() {}

    @Override
    public void updateInputs(TrayIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setPosition(double newValue) {}

    @Override
    public void setMotionMagic(MotionMagicVoltage request) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void setVoltage(Voltage voltage) {}

    @Override
    public void close() throws Exception {}
}

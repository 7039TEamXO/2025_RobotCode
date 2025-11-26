package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.IO.WristIO;

public class WristStub implements WristIO {
    public WristStub() {}

    @Override
    public void updateInputs(WristIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setPosition(double newValue) {}

    @Override
    public void setMotionMagic(DutyCycleOut request) {}

    @Override
    public void setMotionMagic(MotionMagicVoltage request) {}

    @Override
    public void setVoltage(Voltage voltage) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}

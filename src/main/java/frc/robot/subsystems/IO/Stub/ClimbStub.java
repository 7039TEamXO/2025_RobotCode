package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.subsystems.IO.ClimbIO;

public class ClimbStub implements ClimbIO {
    public ClimbStub() {}

    @Override
    public void updateInputs(ClimbIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setPosition(double newValue) {}

    @Override
    public void setMotionMagic(DutyCycleOut request) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}

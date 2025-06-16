package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Climb.ClimbConstants;
import frc.robot.subsystems.IO.ClimbIO;

public class ClimbReal implements ClimbIO {
    private final TalonFX master;

    public ClimbReal() {        
        master = new TalonFX(ClimbConstants.ClimbMotorID);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.position = master.getPosition().getValueAsDouble();
        inputs.velocity = master.getVelocity().getValueAsDouble();
        inputs.appliedVolts = master.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = master.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        master.getConfigurator().apply(configuration);
    }

    @Override
    public void setPosition(double newValue) {
        master.setPosition(newValue);
    }

    @Override
    public void setMotionMagic(DutyCycleOut request) {
        master.setControl(request);
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        master.close();
    }
}

package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.IO.TrayIO;
import frc.robot.subsystems.Tray.TrayConstants;

public class TrayReal implements TrayIO {
    private final TalonFX master;

    public TrayReal() {        
        master = new TalonFX(TrayConstants.TrayMotorID);
    }

    @Override
    public void updateInputs(TrayIOInputs inputs) {
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
    public void setMotionMagic(MotionMagicVoltage request) {
        master.setControl(request);
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        master.close();
    }
}

package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.Handler.HandlerConstants;
import frc.robot.subsystems.IO.HandlerIO;

public class HandlerReal implements HandlerIO {
    private final TalonFX master;

    private final AnalogInput algaeProcIRInput;
    private final AnalogInput algaeNetIrInput;
    
    private final DigitalInput coralIRInput;
    private final DigitalOutput coralIROutput;

    public HandlerReal() {        
        master = new TalonFX(HandlerConstants.HandlerMotorID);

        algaeProcIRInput = new AnalogInput(HandlerConstants.HandlerAnalogProcInputSensorID);
        algaeNetIrInput = new AnalogInput(HandlerConstants.HandlerAnalogNetInputSensorID);
        coralIRInput = new DigitalInput(HandlerConstants.HandlerDigitalInputSensorID);
        coralIROutput = new DigitalOutput(HandlerConstants.HandlerDigitalOutputSensorID);

        coralIROutput.set(true);
    }

    @Override
    public void updateInputs(HandlerIOInputs inputs) {
        inputs.position = master.getPosition().getValueAsDouble();
        inputs.velocity = master.getVelocity().getValueAsDouble();
        inputs.appliedVolts = master.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = master.getStatorCurrent().getValueAsDouble();

        inputs.coralIR = coralIRInput.get();
        inputs.algaeProcessorIR = algaeProcIRInput.getValue();
        inputs.algaeNetIR = algaeNetIrInput.getValue();
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
    public double getStatorCurrent() {
        return master.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        master.close();
        algaeNetIrInput.close();
        algaeProcIRInput.close();
        coralIRInput.close();
        coralIROutput.close();
    }
}

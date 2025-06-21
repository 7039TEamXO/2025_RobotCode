package frc.robot.subsystems.IO.Sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Handler.HandlerConstants;
import frc.robot.subsystems.IO.HandlerIO;

public class HandlerSim implements HandlerIO {
    private final TalonFX master;
    private final TalonFXSimState masterSim;
    private final DCMotorSim motorSimModel;

    private final AnalogInput algaeProcIRInput;
    private final SimDevice algaeProcIRInputSim;
    private final SimInt algaeProcIRInputSimValue;

    private final AnalogInput algaeNetIRInput;
    private final SimDevice algaeNetIRInputSim;
    private final SimInt algaeNetIRInputSimValue;

    private final DigitalInput coralIRInput;
    private final SimDevice coralIRInputSim;
    private final SimBoolean coralIRInputSimValue;

    private final double periodicDt;

    private static final double GearRatio = 10.0;

    public HandlerSim(double _periodicDt) {
        periodicDt = _periodicDt;

        master = new TalonFX(HandlerConstants.HandlerMotorID);
        masterSim = master.getSimState();
        motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, GearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );

        algaeProcIRInput = new AnalogInput(HandlerConstants.HandlerAnalogProcInputSensorID);
        algaeProcIRInputSim = SimDevice.create("AnalogInput0", HandlerConstants.HandlerAnalogProcInputSensorID);
        algaeProcIRInputSimValue = algaeProcIRInputSim.createInt("Value", edu.wpi.first.hal.SimDevice.Direction.kOutput, -1);
        algaeProcIRInput.setSimDevice(algaeProcIRInputSim);

        algaeNetIRInput = new AnalogInput(HandlerConstants.HandlerAnalogNetInputSensorID);
        algaeNetIRInputSim = SimDevice.create("AnalogInput1", HandlerConstants.HandlerAnalogNetInputSensorID);
        algaeNetIRInputSimValue = algaeNetIRInputSim.createInt("Value", edu.wpi.first.hal.SimDevice.Direction.kOutput, -1);
        algaeNetIRInput.setSimDevice(algaeNetIRInputSim);

        coralIRInput = new DigitalInput(HandlerConstants.HandlerDigitalInputSensorID);
        coralIRInputSim = SimDevice.create("DigitalInput", HandlerConstants.HandlerDigitalInputSensorID);
        coralIRInputSimValue = coralIRInputSim.createBoolean("Value", edu.wpi.first.hal.SimDevice.Direction.kOutput, false);
        coralIRInput.setSimDevice(coralIRInputSim);
    }

    @Override
    public void updateInputs(HandlerIOInputs inputs) {
        inputs.position = master.getPosition().getValueAsDouble();
        inputs.velocity = master.getVelocity().getValueAsDouble();
        inputs.appliedVolts = master.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = master.getStatorCurrent().getValueAsDouble();

        inputs.coralIR = coralIRInputSimValue.get();
        inputs.algaeProcessorIR = algaeProcIRInputSimValue.get();
        inputs.algaeNetIR = algaeNetIRInputSimValue.get();
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
    public void simulationPeriodic() {
        masterSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        var motorVoltage = masterSim.getMotorVoltage();

        motorSimModel.setInputVoltage(motorVoltage);
     
        motorSimModel.update(periodicDt);
    
        masterSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(GearRatio));
        masterSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(GearRatio));
    }

    @Override
    public void close() throws Exception {
        master.close();
        
        algaeNetIRInput.close();
        algaeProcIRInput.close();
        coralIRInput.close();
        algaeNetIRInputSim.close();
        algaeProcIRInputSim.close();
        coralIRInputSim.close();
    }
}

package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

public interface HandlerIO extends AutoCloseable {
    public static class HandlerIOInputs implements LoggableInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public boolean coralIR = false;
        public int algaeProcessorIR = -1;
        public int algaeNetIR = -1;

        public void toLog(LogTable table) {
            table.put("Position", position);
            table.put("Velocity", velocity);
            table.put("AppliedVolts", appliedVolts);
            table.put("CurrentAmps", currentAmps);

            table.put("CoralIR", coralIR);
            table.put("AlgaeProcessorIR", algaeProcessorIR);
            table.put("AlgaeNetIR", algaeNetIR);
        }

        public void fromLog(LogTable table) {
            position = table.get("Position", 0.0);
            velocity = table.get("Velocity", 0.0);
            appliedVolts = table.get("AppliedVolts", 0.0);
            currentAmps = table.get("CurrentAmps", 0.0);

            coralIR = table.get("CoralIR", false);
            algaeProcessorIR = table.get("AlgaeProcessorIR", -1);
            algaeNetIR = table.get("AlgaeNetIR", -1);
        }
    }

    public void updateInputs(HandlerIOInputs inputs);

    public void applyTalonFXConfig(TalonFXConfiguration configuration);

    public void setPosition(double newValue);

    public void setMotionMagic(DutyCycleOut request);

    public double getStatorCurrent();

    public void simulationPeriodic();
}

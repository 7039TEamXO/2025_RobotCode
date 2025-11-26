package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.units.measure.Voltage;

public interface ClimbIO extends AutoCloseable {
    public static class ClimbIOInputs implements LoggableInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public void toLog(LogTable table) {
            table.put("Position", position);
            table.put("Velocity", velocity);
            table.put("AppliedVolts", appliedVolts);
            table.put("CurrentAmps", currentAmps);
        }

        public void fromLog(LogTable table) {
            position = table.get("Position", 0.0);
            velocity = table.get("Velocity", 0.0);
            appliedVolts = table.get("AppliedVolts", 0.0);
            currentAmps = table.get("CurrentAmps", 0.0);
        }
    }

    public void updateInputs(ClimbIOInputs inputs);

    public void applyTalonFXConfig(TalonFXConfiguration configuration);

    public void setPosition(double newValue);

    public void setMotionMagic(DutyCycleOut request);

    public void setVoltage(Voltage voltage);

    public void simulationPeriodic();
}

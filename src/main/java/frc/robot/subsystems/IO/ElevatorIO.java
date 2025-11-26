package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO extends AutoCloseable {
    public static class ElevatorIOInputs implements LoggableInputs {
        public double masterPosition = 0.0;
        public double masterVelocity = 0.0;
        public double masterAppliedVolts = 0.0;
        public double masterCurrentAmps = 0.0;

        public double slavePosition = 0.0;
        public double slaveVelocity = 0.0;
        public double slaveAppliedVolts = 0.0;
        public double slaveCurrentAmps = 0.0;

        public void toLog(LogTable table) {
            table.put("MasterPosition", masterPosition);
            table.put("MasterVelocity", masterVelocity);
            table.put("MasterAppliedVolts", masterAppliedVolts);
            table.put("MasterCurrentAmps", masterCurrentAmps);
            table.put("SlavePosition", slavePosition);
            table.put("SlaveVelocity", slaveVelocity);
            table.put("SlaveAppliedVolts", slaveAppliedVolts);
            table.put("SlaveCurrentAmps", slaveCurrentAmps);
        }

        public void fromLog(LogTable table) {
            masterPosition = table.get("MasterPosition", 0.0);
            masterVelocity = table.get("MasterVelocity", 0.0);
            masterAppliedVolts = table.get("MasterAppliedVolts", 0.0);
            masterCurrentAmps = table.get("MasterCurrentAmps", 0.0);
            slavePosition = table.get("SlavePosition", 0.0);
            slaveVelocity = table.get("SlaveVelocity", 0.0);
            slaveAppliedVolts = table.get("SlaveAppliedVolts", 0.0);
            slaveCurrentAmps = table.get("SlaveCurrentAmps", 0.0);
        }
    }

    public void updateInputs(ElevatorIOInputs inputs);

    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration);

    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration);

    public Follower createFollower();

    public void setLeadMotorPosition(double newValue);

    public void setFollowerMotorPosition(double newValue);

    public void setLeadMotionMagic(MotionMagicExpoVoltage request);

    public void setFollowerMotionMagic(Follower request);

    public void setVoltage(Voltage voltage);

    public void simulationPeriodic();
}
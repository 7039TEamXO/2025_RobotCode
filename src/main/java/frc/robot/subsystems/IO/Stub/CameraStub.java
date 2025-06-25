package frc.robot.subsystems.IO.Stub;

import frc.robot.subsystems.IO.CameraIO;

public class CameraStub implements CameraIO {
    public CameraStub() {}

    @Override
    public void init() {}

    @Override
    public void updateInputs(CameraIOInputs inputs) {}

    @Override
    public void periodic() {}

    @Override
    public void close() throws Exception {
        // ...
    }
}
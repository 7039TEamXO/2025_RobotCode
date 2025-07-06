package frc.robot;

public enum Autos {
    Red_Right_Coral("Red Right Coral"),
    Blue_Right_Coral("Blue Right Coral"),
    Red_Left_Coral("Red Left Coral"),
    Blue_Left_Coral("Blue Left Coral");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}

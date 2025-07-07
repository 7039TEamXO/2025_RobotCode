package frc.robot;

public enum Autos {
    Blue_Right_Coral("Blue Right Coral"),
    Red_Right_Coral("Red Right Coral"),
    Blue_Left_Coral("Blue Left Coral"),
    Red_Left_Coral("Red Left Coral");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}

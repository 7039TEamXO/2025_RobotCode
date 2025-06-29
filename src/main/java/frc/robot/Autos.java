package frc.robot;

public enum Autos {
    Safe_Left("Safe_Left"),
    Safe_Right("Safe_Right"),
    Level_4_Left("Level_4_Left"),
    Level_4_Right("Level_4_Right"),
    Push_Left("Push_Left"),
    Double_Algae_Right("Double_Algae_Right"),
    Coral_Algae_Right("Coral_Algae_Right");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}

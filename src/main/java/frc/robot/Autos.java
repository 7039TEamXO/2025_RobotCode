package frc.robot;

public enum Autos {
    Test("Test"),
    Safe_Left("Safe_Left"),
    Safe_Right("Safe_Right"),
    Level_4_Left("Level_4_Left"),
    Level_4_Right("Level_4_right"),
    Double_Algae_Left("Double_Algae_Left"),
    Double_Algae_Right("Double_Algae_Right"),
    Coral_Algae_Left("Coral_Algae_Left"),
    Coral_Algae_Right("Coral_Algae_Right");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}

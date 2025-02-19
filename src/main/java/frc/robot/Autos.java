package frc.robot;

public enum Autos {
    Safe_Right("Safe_Right"),
    Level_4_Right("Level_4_Right"),
    Safe_Left("Safe_Left"),
    Level_4_Left("Level_4_Left"),
    Double_Algae_Right("Double_Algae_Right"),
    Double_Algae_Left("Double_Algae_Left"),
    Test("Test");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}

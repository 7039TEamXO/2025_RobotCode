package frc.robot;

public enum Autos {
    Test("Test"),
    Safe_Left("Safe_Left"),
    Safe_Right("Safe_Right"),
    Level_4_Left_Red("Level_4_Left_Red"),
    Level_4_Left_Blue_Copy("Level_4_Left_Blue_Copy"),
    Level_4_Right_Red("Level_4_Right_Red"),
    Level_4_Right_Blue("Level_4_Right_Blue"),
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

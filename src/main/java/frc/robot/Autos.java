package frc.robot;

public enum Autos {
    MID_AUTO("ExampleAuto"),
    LEFT_AUTO("LeftAuto"),
    HOME_RED_START("HomeRedStart");
    
    private final String autoName;
    
    Autos(final String autoName){
        this.autoName = autoName;
    }

    public String getAutoName(){
        return autoName;
    }
}

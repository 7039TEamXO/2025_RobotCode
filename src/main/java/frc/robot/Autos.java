package frc.robot;

public enum Autos {
    TripleCoralDownAuto("TripleCoralDownAuto"),
    TripleCoralDownLevel3Auto("TripleCoralDownV2Auto"),
    TripleCoralUpAuto("TripleCoralUpAuto"),
    TripleCoralUpLevel3Auto("TripleCoralUpV2Auto"),
    DoubleAlgaeDownAuto("DoubleAlgaeDownAuto"),
    DoubleAlgaeUpAuto("DoubleAlgaeUpAuto");
    
    private final String autoName;
    
    Autos(final String autoName){
        this.autoName = autoName;
    }

    public String getAutoName(){
        return autoName;
    }
}

package frc.robot.constants;

public enum IntakeSpeeds {

    COLLECT(0.5,0.7),
    STOP(0,0),
    FEED(-1,1),
    REJECT(-0.2,0),
    PANIC(-1,-1);

    public double roller;
    public double track;
    IntakeSpeeds(double roller, double track){
        this.roller = roller;
        this.track = track;
    }
}

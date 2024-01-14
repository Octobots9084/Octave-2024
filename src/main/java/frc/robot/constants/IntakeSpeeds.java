package frc.robot.constants;

public enum IntakeSpeeds {

    COLLECT(1,1),
    STOP(0,0),
    FEED(-1,1),
    REJECT(-1,0),
    PANIC(-1,-1);

    double roller, track;
    IntakeSpeeds(double roller, double track){
        this.roller = roller;
        this.track = track;
    }
}

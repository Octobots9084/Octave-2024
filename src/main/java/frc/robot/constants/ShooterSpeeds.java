package frc.robot.constants;

public enum ShooterSpeeds {
    IDLE(0,-0.5),
    PREPARE(0,1),
    SPEAKER(0.7,-2500),
    STOP(0,0),
    PANIC(-1,-1),
    AMP(1,-1000),
    JIGGLE_FORWARD(1, 0),
    JIGGLE_BACKWARD(-1, 0),
    TRAP(1,0.5);

    public double track;
	public double flywheels;
    ShooterSpeeds(double track, double flywheels) {
        this.track = track;
        this.flywheels = flywheels;
    }
}

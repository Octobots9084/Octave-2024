package frc.robot.constants;

public enum ShooterSpeeds {
    IDLE(0,0.5),
    PREPARE(0,1),
    SPEAKER(1,1),
    STOP(0,0),
    PANIC(-1,-1),
    AMP(1,0.5),
    TRAP(1,0.5);
    public double track;
	public double flywheels;
    ShooterSpeeds(double track, double flywheels) {
        this.track = track;
        this.flywheels = flywheels;
    }
}

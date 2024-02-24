package frc.robot.constants;

public enum ShooterSpeeds {
    IDLE(0, -0.5),
    PREPARE(0.2, 1),
    SPEAKER(0.7, -2500),
    STOP(0, 0),
    PANIC(1, -500),
    AMP(0.7, -2500),
    JIGGLE_FORWARD(0.2, 0),
    JIGGLE_BACKWARD(-0.2, 0),
    TRAP(1, 0.5),
    DRIVE_BY(0, -20);

    public double track;
    public double flywheels;

    ShooterSpeeds(double track, double flywheels) {
        this.track = track;
        this.flywheels = flywheels;
    }
}
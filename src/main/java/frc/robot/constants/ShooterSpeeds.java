package frc.robot.constants;

public enum ShooterSpeeds {
    IDLE(0, -0.5),
    PREPARE(0.15, 1),
    SPEAKER(0.7, -20),
    STOP(0, 0),
    PANIC(1, -500),
    AMP(1, -20),
    JIGGLE_FORWARD(0.15, 0),
    JIGGLE_BACKWARD(-0.15, 0),
    TRAP(1, 0.5),
    SPECIAL_IDLE(0.15, 0),
    DRIVE_BY(0, -20), AUTO_SPEAKER(0, -10);

    public double track;
    public double flywheels;

    ShooterSpeeds(double track, double flywheels) {
        this.track = track;
        this.flywheels = flywheels;
    }
}

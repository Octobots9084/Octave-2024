package frc.robot.constants;

public enum ShooterSpeeds {
    IDLE(0.1, -0.5),
    PREPARE(0.1, 1),
    SPEAKER(0.7, -20),
    STOP(0, 0),
    PANIC(1, -500),
    AMP(1, -20),
    FERRY_SHOT(1, -20),
    JIGGLE_FORWARD(0.1, 0),
    JIGGLE_BACKWARD(-0.1, 0),
    TRAP(1, -20),
    LAYUP(-0.05, -5),
    SPECIAL_IDLE(0.1, 0),
    DRIVE_BY(0, -20),
    SRC_COLLECT(-.3, -1),
    AUTO_SPEAKER(0, -20);

    public double track;
    public double flywheels;

    ShooterSpeeds(double track, double flywheels) {
        this.track = track;
        this.flywheels = flywheels;
    }
}

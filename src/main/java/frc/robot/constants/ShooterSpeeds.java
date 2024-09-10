package frc.robot.constants;

public enum ShooterSpeeds {
    IDLE(0.1, -0.5),
    PREPARE(2.5, 1),
    PREPARE_AUTO(.2, 1),
    SPEAKER(0.7, -20),
    STOP(0, 0),
    PANIC(1, -500),
    AMP(1, -7),
    FERRY_SHOT(1, -30),
    JIGGLE_FORWARD(1, 0),
    JIGGLE_BACKWARD(-1, 0),
    TRAP(1, -30),
    LAYUP(-0.2, -5),
    SPECIAL_IDLE(0.1, 0),
    DRIVE_BY(0, -20),
    SRC_COLLECT(-.3, -1),
    AUTO_SPEAKER(0, -20),
    REVERSE_TRACK(-1, 0);

    public double track;
    public double flywheels;

    ShooterSpeeds(double track, double flywheels) {
        this.track = track;
        this.flywheels = flywheels;
    }
}

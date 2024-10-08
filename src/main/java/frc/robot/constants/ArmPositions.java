package frc.robot.constants;

public enum ArmPositions {
    HANDOFF_AND_DEFAULT_SHOT(0, 0.453), // original pivot: 0.443
    AMP(22.5, 0.73),
    TRAP_SEGUEAY(5, 0.39),
    TRAP(44.8, 0.64),
    LAYUP(8.75, 0.93), // prev pivot: 0.89
    FERRY_SHOT(0, 0.597),
    SOURCE_COLLECT(0, 0), // placeholder
    SPEAKER_SHOT(0, 0.462),
    PREP_TRAP(15, 0.39),
    INITAL_AUTO(15, 0.45),
    TUNE(0, 0.691 - 0.072);

    public double elevator, pivot;

    ArmPositions(double elevator, double pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }
}
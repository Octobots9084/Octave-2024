package frc.robot.constants;

public enum ArmPositions {
    HANDOFF_AND_DEFAULT_SHOT(0, 0.435),
    AMP(20, 0.69),
    TRAP_SEGUEAY(20, 0.39),
    TRAP(44.8, 0.69),
    LAYUP(5,0.888),
    SOURCE_COLLECT(0,0), //placeholder
    SPEAKER_SHOT(0, 0.46);

    public double elevator, pivot;

    ArmPositions(double elevator, double pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }
}
package frc.robot.constants;

public enum ClimbPositions {
    DOWN(0, 0),
    MID(2, 2),
    UP(4.3, 4.3), LAYUP(0.52, 0.52);

    public double leftPosition;
    public double rightPosition;

    ClimbPositions(double leftPosition, double rightPosition) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
    }
}

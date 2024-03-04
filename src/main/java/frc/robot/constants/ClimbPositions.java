package frc.robot.constants;

public enum ClimbPositions {
    DOWN(0, 0),
    MID(2, 2),
    UP(4, 4);

    public double leftPosition;
    public double rightPosition;

    ClimbPositions(double leftPosition, double rightPosition) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
    }
}

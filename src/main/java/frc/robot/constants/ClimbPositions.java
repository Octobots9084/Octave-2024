package frc.robot.constants;

public enum ClimbPositions {
    DOWN(0, 0),
    MID(2, 2),
    UP(3.95, 3.95);

    public double leftPosition;
    public double rightPosition;

    ClimbPositions(double leftPosition, double rightPosition) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
    }
}

package frc.robot.constants;

public enum ClimbPositions {
    DOWN(0, 0),
    MID(0.5, 0.5),
    UP(3.6, 3.6);

    public double leftPosition;
    public double rightPosition;

    ClimbPositions(double leftPosition, double rightPosition) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
    }
}

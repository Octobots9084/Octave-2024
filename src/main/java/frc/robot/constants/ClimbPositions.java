package frc.robot.constants;

public enum ClimbPositions {
    DOWN(0),
    MID(0.5),
    UP(30);

    public double position;
    ClimbPositions(double position) {
        this.position = position;
    }
}




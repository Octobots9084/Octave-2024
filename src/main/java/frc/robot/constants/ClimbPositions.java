package frc.robot.constants;

public enum ClimbPositions {
    DOWN(0),
    MID(0.5),
    UP(1);

    public double position;
    ClimbPositions(double position) {
        this.position = position;
    }
}




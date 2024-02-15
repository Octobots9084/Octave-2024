package frc.robot.constants;

    public enum ArmPositions {
        HANDOFF_AND_DEFAULT_SHOT(0,0.43),
        AMP(20,0.74),
        TRAP_SEGUEAY(0,0),
        TRAP(30,0);
        
        public double elevator, pivot;
    ArmPositions(double elevator, double pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }
}
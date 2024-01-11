package frc.robot.util.SparkMax;

import java.util.ArrayList;

public class SparkMaxStatusFrames {
    ArrayList<Integer> statusFrames = new ArrayList<Integer>();

    /**
     * Create a SparkMaxStatusFrames object
     * 
     * @param periodicStatus0 Applied **** Output, Faults, Sticky Faults, Is
     *                        Follower
     * @param periodicStatus1 Motor Velocity, Motor Temperature, Motor Voltage,
     *                        Motor Current
     * @param periodicStatus2 Motor Position
     * @param periodicStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog
     *                        Sensor Position
     * @param periodicStatus4 Alternate Encoder Velocity, Alternate Encoder Position
     * @param periodicStatus5 Duty Cycle Absolute Encoder Position, Duty Cycle
     *                        Absolute Encoder Absolute Angle
     * @param periodicStatus6 Duty Cycle Absolute Encoder Velocity, Duty Cycle
     *                        Absolute Encoder Frequency
     */
    public SparkMaxStatusFrames(int periodicStatus0, int periodicStatus1, int periodicStatus2,
            int periodicStatus3, int periodicStatus4, int periodicStatus5, int periodicStatus6) {
        this.statusFrames.add(periodicStatus0);
        this.statusFrames.add(periodicStatus1);
        this.statusFrames.add(periodicStatus2);
        this.statusFrames.add(periodicStatus3);
        this.statusFrames.add(periodicStatus4);
        this.statusFrames.add(periodicStatus5);
        this.statusFrames.add(periodicStatus6);

    }

    public ArrayList<Integer> getFrames() {
        return this.statusFrames;
    }
}

package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.constants.AlignHotSpots;

public class ClimbAlign extends BaseAlign {

    public ClimbAlign() {
        super();
    }

    @Override
    protected AlignHotSpots[] getHotSpots() {
        return new AlignHotSpots[] { Constants.isBlueAlliance ? AlignHotSpots.BlueClimbOne
                : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueClimbOne),
                Constants.isBlueAlliance ? AlignHotSpots.BlueClimbTwo
                        : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueClimbTwo),
                Constants.isBlueAlliance ? AlignHotSpots.BlueClimbThree
                        : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueClimbThree) };
    }
}

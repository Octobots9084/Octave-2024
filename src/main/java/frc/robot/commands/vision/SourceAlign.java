package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.constants.AlignHotSpots;

public class SourceAlign extends BaseAlign {

    public SourceAlign() {
        super();
    }

    @Override
    protected AlignHotSpots[] getHotSpots() {
        return new AlignHotSpots[] { Constants.isBlueAlliance ? AlignHotSpots.BlueSource
                : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueSource) };
    }
}

package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.constants.AlignHotSpots;

public class AmpAlign extends BaseAlign {

    public AmpAlign() {
        super();
    }

    @Override
    protected AlignHotSpots[] getHotSpots() {
        return new AlignHotSpots[] { Constants.isBlueAlliance ? AlignHotSpots.BlueAmp
                : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueAmp) };
    }
}

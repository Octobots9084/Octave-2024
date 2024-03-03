package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.complex.Collect;
import frc.robot.commands.complex.CollectAuto2;
import frc.robot.commands.complex.Driveby;
import frc.robot.commands.complex.PrepSpeaker;
import frc.robot.commands.complex.TheBigYeet;

public class ParallelAutoHandling extends SequentialCommandGroup{
    public ParallelAutoHandling() {
        addCommands(new RepeatCommand(new ParallelCommandGroup(new CollectAuto2(), new Driveby())));
    };
}

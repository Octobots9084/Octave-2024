package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCollect extends SequentialCommandGroup {
  public AutoCollect() {
    addCommands(
      new ParallelDeadlineGroup(new Collect(), new DriveToNote())
        

    );
  }
}

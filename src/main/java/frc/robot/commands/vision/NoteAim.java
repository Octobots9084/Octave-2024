package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.PieceVision;

public class NoteAim extends Command {
    private PieceVision pieceCam;
    private SwerveSubsystem swerve;

    public NoteAim() {
        pieceCam = PieceVision.getInstance();
        swerve = SwerveSubsystem.getInstance();
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(0, 0), pieceCam.getBestPiece().getYaw(), true);
    }
}

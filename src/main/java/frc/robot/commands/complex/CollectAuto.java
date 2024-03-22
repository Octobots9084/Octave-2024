package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.commands.intake.IntakeRollerSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class CollectAuto extends SequentialCommandGroup {
    public CollectAuto() {
        // SmartDashboard.putNumber("Collect", SmartDashboard.getNumber("Collect", 0) +
        // 1);
        System.out.println("collectconst");
        BooleanSupplier intakeSensorTrue = () -> !IntakeTrack.getInstance().getAnalogDigital();
        BooleanSupplier rollerSensor = () -> {
            return (!IntakeRoller.getInstance().getSensor() || !IntakeTrack.getInstance().getSensor2());
        };
        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorNotTrue = () -> ShooterTrack.getInstance().getSensor();
        addCommands(
                new InstantCommand(() -> {
                    System.out.println("collect stage 0");
                }),
                new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.COLLECTING);
                }),

                new ShooterTrackSpeedInstant(ShooterSpeeds.IDLE),
                new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT),
                new WaitUntilCommand(intakeSensorTrue),
                new InstantCommand(() -> {
                    System.out.println("collect stage 1");
                }),
                new IntakeRollerSpeedInstant(IntakeSpeeds.STOP),
                new IntakeTrackSpeedInstant(IntakeSpeeds.STOP),
                new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT).withTimeout(2.5),
                new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT),
                new ShooterTrackSpeedInstant(ShooterSpeeds.PREPARE_AUTO),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(shooterSensorTrue),
                                new InstantCommand(() -> {
                                    System.out.println("collect stage 2");
                                }),
                                new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
                                new IntakeTrackSpeedInstant(IntakeSpeeds.STOP),
                                new InstantCommand(() -> {
                                    ShooterPivot.getInstance().notSoFastEggman = false;
                                }),
                                new InstantCommand(() -> {
                                    Light.getInstance().setAnimation(Animations.PRE_INTAKE);
                                }),
                                new InstantCommand(() -> {
                                    System.out.println("collect stage 2.1");
                                })),
                        new InstantCommand(() -> {
                            System.out.println("collect stage 2.2");
                        }),
                        new WaitCommand(0.2).andThen(new IntakeRollerSpeedInstant(IntakeSpeeds.STOP))),
                new InstantCommand(() -> {
                    System.out.println("collect stage 3");
                })

        );
    }

}

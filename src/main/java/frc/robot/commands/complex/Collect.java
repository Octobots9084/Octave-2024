package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.JiggleNote;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
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
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class Collect extends SequentialCommandGroup {
    public Collect() {

        BooleanSupplier intakeSensorTrue = () -> !IntakeTrack.getInstance().getAnalogDigital();
        BooleanSupplier rollerSensor = () -> {
            return (!IntakeRoller.getInstance().getSensor() || !IntakeTrack.getInstance().getSensor2());
        };
        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorNotTrue = ()->{return (ShooterTrack.getInstance().getSensor()||!IntakeTrack.getInstance().getAnalogDigital());};
        addCommands(
                new ParallelDeadlineGroup(new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(() -> {
                    ShooterPivot.getInstance().notSoFastEggman = true;
                }),
                        new InstantCommand(() -> {
                            System.out.println("Collect requested");
                        }),
                        new InstantCommand(() -> {
                            Light.getInstance().setAnimation(Animations.COLLECTING);
                        }),
                        new InstantCommand(()->{
                            ShooterFlywheel.getInstance().setFlyWheelSpeedMeters(ShooterSpeeds.SPEAKER.flywheels);
                        }),

                        new ShooterTrackSpeedInstant(ShooterSpeeds.IDLE),
                        new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT),
                        new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT).withTimeout(2.5),
                        new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT).withTimeout(0.1),
                        new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                        new WaitUntilCommand(intakeSensorTrue),
                        new InstantCommand(() -> {
                            System.out.println("Intake track sensor tripped");
                        }),
                        new IntakeRollerSpeedInstant(IntakeSpeeds.STOP),
                        new IntakeTrackSpeedInstant(IntakeSpeeds.STOP),
                        new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT).withTimeout(2.5),
                        new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT).withTimeout(0.1),
                        new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                        new IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT),
                        new ShooterTrackSpeedInstant(ShooterSpeeds.PREPARE),
                        new ParallelCommandGroup(new SequentialCommandGroup(
                                new WaitUntilCommand(shooterSensorTrue),
                                new InstantCommand(() -> {
                                    System.out.println("Shooter sensor tripped");
                                }),
                                new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
                                new IntakeTrackSpeedInstant(IntakeSpeeds.STOP),
                                new InstantCommand(() -> {
                                    ShooterPivot.getInstance().notSoFastEggman = false;
                                }),
                                new PrepSpeaker(),
                                new InstantCommand(() -> {
                                    Light.getInstance().setAnimation(Animations.PRE_INTAKE);
                                }),
                                new JiggleNote(.5), new InstantCommand(() -> {
                                    Light.getInstance().setAnimation(Animations.INTAKE_STAGE_2);
                                }),
                                new InstantCommand(() -> {
                                    System.out.println("Collect Complete");
                                })),
                                new WaitCommand(0.2).andThen(new IntakeRollerSpeedInstant(IntakeSpeeds.STOP)))),
                                new SequentialCommandGroup(new InstantCommand(() -> {
                                    Light.getInstance().setAnimation(Animations.JADEN_U_HAVE_A_NOTE);
                                }),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> {
                                    Light.getInstance().setAnimation(Animations.INTAKE_STAGE_2);
                                })), shooterSensorNotTrue),
                        new WaitUntilCommand(rollerSensor).andThen(new InstantCommand(() -> {
                            Light.getInstance().setAnimation(Animations.INTAKE_STAGE_1);
                        })).andThen(new InstantCommand(() -> {
                            System.out.println("Roller sensors tripped");
                        })))

        );
    }

}

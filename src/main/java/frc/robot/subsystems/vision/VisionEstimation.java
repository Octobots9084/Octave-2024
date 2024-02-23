/*
 * This file is part of Placeholder-2023, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Octobots <https://github.com/Octobots9084>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package frc.robot.subsystems.vision;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.telemetry.CountPerPeriodTelemetry;
import frc.robot.util.telemetry.MeanPerPeriodTelemetry;

public class VisionEstimation extends SubsystemBase {
        private final SwerveSubsystem swerveSubsystem;

        public Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9,
                        0.9,
                        0.9);

        private final Vision backRightEstimator = new Vision(new PhotonCamera("Inky"),
                        VisionConstants.ROBOT_TO_INKY);
        // private final Vision frontLeftEstimator = new Vision(new
        // PhotonCamera("Blinky"),
        // VisionConstants.ROBOT_TO_BLINKY);
        private final Vision backLeftEstimator = new Vision(new PhotonCamera("Pinky"),
                        VisionConstants.ROBOT_TO_PINKY);
        // private final Vision backLeftEstimator = new Vision(new
        // PhotonCamera("Clyde"),
        // VisionConstants.ROBOT_TO_CLYDE);

        private final Notifier allNotifier = new Notifier(() -> {
                backRightEstimator.run();
                // frontLeftEstimator.run();
                // backRightEstimator.run();
                backLeftEstimator.run();
        });

        private OriginPosition originPosition = kBlueAllianceWallRightSide;

        // Telemetry
        private final CountPerPeriodTelemetry runCountTelemetry;

        private final CountPerPeriodTelemetry getAtomicCountInkyTelemetry;
        // private final CountPerPeriodTelemetry getAtomicCountBlinkyTelemetry;
        private final CountPerPeriodTelemetry getAtomicCountPinkyTelemetry;
        // private final CountPerPeriodTelemetry getAtomicCountClydeTelemetry;

        private final MeanPerPeriodTelemetry confidenceInkyTelemetry;
        // private final MeanPerPeriodTelemetry confidenceBlinkyTelemetry;
        private final MeanPerPeriodTelemetry confidencePinkyTelemetry;
        //private final MeanPerPeriodTelemetry confidenceClydeTelemetry;

        public VisionEstimation() {
                this.swerveSubsystem = SwerveSubsystem.getInstance();

                allNotifier.setName("runAll");
                allNotifier.startPeriodic(0.02);

                // Initialize telemetry
                runCountTelemetry = new CountPerPeriodTelemetry("Vision/Estimation - runs per s", 1);

                getAtomicCountInkyTelemetry = new CountPerPeriodTelemetry("Vision/Inky - meas per s pull", 1);
                // getAtomicCountBlinkyTelemetry = new CountPerPeriodTelemetry("Vision/Blinky - meas per s pull", 1);
                getAtomicCountPinkyTelemetry = new CountPerPeriodTelemetry("Vision/Pinky - meas per s pull", 1);
                // getAtomicCountClydeTelemetry = new CountPerPeriodTelemetry("Vision/Clyde - meas per s pull", 1);


                confidenceInkyTelemetry = new MeanPerPeriodTelemetry("Vision/Inky - confidence mult", 0.5, -999.99);
                // confidenceBlinkyTelemetry = new MeanPerPeriodTelemetry("Vision/Blinky - confidence mult", 0.5, -999.99);
                confidencePinkyTelemetry = new MeanPerPeriodTelemetry("Vision/Pinky - confidence mult", 0.5, -999.99);
                // confidenceClydeTelemetry = new MeanPerPeriodTelemetry("Vision/Clyde - confidence mult", 0.5, -999.99);
        }

        @Override
        public void periodic() {
                if (VisionConstants.USE_VISION) {
                        // Update "run count" telemetry
                        runCountTelemetry.incCount(1);

                        estimatorChecker(backRightEstimator, getAtomicCountInkyTelemetry, confidenceInkyTelemetry);
                        // estimatorChecker(frontLeftEstimator, getAtomicCountBlinkyTelemetry, confidenceBlinkyTelemetry);
                        estimatorChecker(backRightEstimator, getAtomicCountPinkyTelemetry, confidencePinkyTelemetry);
                        // estimatorChecker(backLeftEstimator, getAtomicCountPinkyTelemetry, confidenceClydeTelemetry);
                } else {
                        allNotifier.close();
                }

                // Run telemetry
                runCountTelemetry.periodic();
                getAtomicCountInkyTelemetry.periodic();
                // getAtomicCountBlinkyTelemetry.periodic();
                getAtomicCountPinkyTelemetry.periodic();
                // getAtomicCountClydeTelemetry.periodic();
                 confidenceInkyTelemetry.periodic();
                // confidenceBlinkyTelemetry.periodic();
                confidencePinkyTelemetry.periodic();
                // confidenceClydeTelemetry.periodic();
        }

        /**
         * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
         * always on the right corner of your
         * alliance wall, the field elements are at different coordinates
         * for each alliance.
         * 
         * @param poseToFlip pose to transform to the other alliance
         * @return pose relative to the other alliance's coordinate system
         */
        private Pose2d flipAlliance(Pose2d poseToFlip) {
                return poseToFlip.relativeTo(new Pose2d(
                                new Translation2d(FieldConstants.LENGTH, FieldConstants.WIDTH),
                                new Rotation2d(Math.PI)));
        }

        private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation, MeanPerPeriodTelemetry confidenceTelemetry) {
                double smallestDistance = Double.POSITIVE_INFINITY;
                for (var target : estimation.targetsUsed) {
                        var target3D = target.getBestCameraToTarget();
                        var distance = Math
                                        .sqrt(Math.pow(target3D.getX(), 2) + Math.pow(target3D.getY(), 2)
                                                        + Math.pow(target3D.getZ(), 2));
                        if (distance < smallestDistance)
                                smallestDistance = distance;
                }

                var maxPoseAmbiguity = Math.max(
                                1,
                                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                                                + VisionConstants.POSE_AMBIGUITY_SHIFTER)
                                                * VisionConstants.POSE_AMBIGUITY_MULTIPLIER);

                double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                                ? 1
                                : maxPoseAmbiguity;

                double confidenceMultiplier = Math.max(
                                1,
                                (Math.max(
                                                1,
                                                Math.max(0, smallestDistance - VisionConstants.NOISY_DISTANCE_METERS)
                                                                * VisionConstants.DISTANCE_WEIGHT)
                                                * poseAmbiguityFactor)
                                                / (1
                                                                + ((estimation.targetsUsed.size() - 1)
                                                                                * VisionConstants.TAG_PRESENCE_WEIGHT)));

                confidenceTelemetry.addNumber(confidenceMultiplier);
                return visionMeasurementStdDevs.times(confidenceMultiplier);
        }

        public void estimatorChecker(Vision estimator, CountPerPeriodTelemetry getAtomicCountTelemetry, MeanPerPeriodTelemetry confidenceTelemetry) {
                var cameraPose = estimator.grabLatestEstimatedPose();

                if (cameraPose != null) {
                        // New pose from vision
                        var pose2d = cameraPose.estimatedPose.toPose2d();
                        if (originPosition == kRedAllianceWallRightSide) {
                                pose2d = flipAlliance(pose2d);
                        }

                        final var confidence = confidenceCalculator(cameraPose, confidenceTelemetry);
                        swerveSubsystem.addVisionReading(pose2d, cameraPose.timestampSeconds, confidence);

                        // Update "get atomic count" telemetry
                        getAtomicCountTelemetry.incCount(1);
                }

                final String smartDashboardCamPoseKey = "Vision/" + estimator.cameraName + " - last pose";
                SmartDashboard.putString(smartDashboardCamPoseKey, cameraPose != null ? cameraPose.estimatedPose.toString() : "no pose");
        }
}
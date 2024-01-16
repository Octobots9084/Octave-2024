package frc.robot.util.SparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class SparkMaxSetup {
    /**
     * Sets up a Spark Max with the supplied config
     * 
     * @param sparkMax       The Spark Max to apply the config to
     * @param sparkMaxConfig The config to apply
     */
    public static void setup(CANSparkMax sparkMax, SparkMaxConfig sparkMaxConfig) {
        if (sparkMaxConfig.getReset()) {
            sparkMax.restoreFactoryDefaults();
        }

        for (int i = 0; i < sparkMaxConfig.getStatusFrames().getFrames().size(); i++) {
            sparkMax.setPeriodicFramePeriod(PeriodicFrame.values()[i],
                    sparkMaxConfig.getStatusFrames().getFrames().get(i));
        }

        sparkMax.setCANTimeout(sparkMaxConfig.getTimeoutMs());

        if (!sparkMaxConfig.isFollower()) {
            if (sparkMaxConfig.getEncoderType() == SparkMaxEncoderType.Absolute) {
                sparkMax.getPIDController().setFeedbackDevice(sparkMax.getAbsoluteEncoder(Type.kDutyCycle));
                sparkMax.getAbsoluteEncoder(Type.kDutyCycle).setInverted(sparkMaxConfig.getSensorInverted());
            } else if (sparkMaxConfig.getEncoderType() == SparkMaxEncoderType.Alternate) {
                sparkMax.getPIDController()
                        .setFeedbackDevice(sparkMax.getAlternateEncoder(sparkMaxConfig.getEncoderCountsPerRev()));
                sparkMax.getAlternateEncoder(sparkMaxConfig.getEncoderCountsPerRev())
                        .setInverted(sparkMaxConfig.getSensorInverted());
            } else {
                sparkMax.getPIDController().setFeedbackDevice(sparkMax.getEncoder());
            }
        }

        sparkMax.setIdleMode(sparkMaxConfig.getIdleMode());
        sparkMax.setInverted(sparkMaxConfig.getInverted());

        sparkMax.setSmartCurrentLimit(sparkMaxConfig.getStallCurrentLimit(), sparkMaxConfig.getFreeCurrentLimit());

        if (!sparkMaxConfig.isFollower()) {
            sparkMax.getPIDController().setPositionPIDWrappingEnabled(sparkMaxConfig.getPositionPIDWrappingEnabled());

            sparkMax.getPIDController().setSmartMotionMaxVelocity(sparkMaxConfig.getMaxVel(), 0);
            sparkMax.getPIDController().setSmartMotionMaxAccel(sparkMaxConfig.getMaxAccel(), 0);
            sparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(
                    sparkMaxConfig.getAllowableClosedLoopError(),
                    0);

            if (sparkMaxConfig.getPIDconfig().getIZone() != null) {
                sparkMax.getPIDController().setIZone(sparkMaxConfig.getPIDconfig().getIZone());
            }

            sparkMax.getPIDController().setP(sparkMaxConfig.getPIDconfig().getP());
            sparkMax.getPIDController().setI(sparkMaxConfig.getPIDconfig().getI());
            sparkMax.getPIDController().setD(sparkMaxConfig.getPIDconfig().getD());
            sparkMax.getPIDController().setFF(sparkMaxConfig.getPIDconfig().getF());
        } else {
            sparkMax.follow(sparkMaxConfig.getLeadSparkMax(), sparkMaxConfig.isInverted());
        }

    }
}

package frc.spectrumLib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism.Config;
import lombok.Getter;

public class SpectrumCANcoder {
    @Getter private int CANcoderID;
    @Getter private double gearRatio = 1;
    @Getter private double offset = 0;
    @Getter private boolean attached = false;

    @Getter private CANcoder canCoder;

    private enum CANCoderFeedbackType {
        RemoteCANcoder,
        FusedCANcoder,
        SyncCANcoder,
    }

    private CANCoderFeedbackType feedbackSource = CANCoderFeedbackType.FusedCANcoder;

    public SpectrumCANcoder(int CANcoderID, TalonFX motor, Config config) {
        this.CANcoderID = CANcoderID;

        if (isAttached()) {
            modifyMotorConfig(
                    motor,
                    config); // Modify configuration to use remote CANcoder fused //TODO: Move below
            // cancoder config and check config worked before adjusting motor
            // @cycIes
            canCoder = new CANcoder(CANcoderID, Rio.CANIVORE);
            CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
            cancoderConfigs.MagnetSensor.MagnetOffset = offset;
            cancoderConfigs.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;
            cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            checkCANcoderResponse(canCoder.getConfigurator().apply(cancoderConfigs));
        }
    }

    public SpectrumCANcoder setGearRatio(double ratio) {
        gearRatio = ratio;
        return this;
    }

    public SpectrumCANcoder setOffset(double offset) {
        this.offset = offset;
        return this;
    }

    public SpectrumCANcoder setAttached(boolean attached) {
        this.attached = attached;
        return this;
    }

    public SpectrumCANcoder modifyMotorConfig(TalonFX motor, Config config) {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration talonConfigMod = config.getTalonConfig();
        talonConfigMod.Feedback.FeedbackRemoteSensorID = CANcoderID;
        switch (feedbackSource) {
            case RemoteCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        talonConfigMod.Feedback.RotorToSensorRatio = gearRatio;
        configurator.apply(talonConfigMod);
        config.setTalonConfig(talonConfigMod);
        return this;
    }

    public void checkCANcoderResponse(StatusCode response) {
        if (!response.isOK()) {
            Telemetry.print(
                    "Pivot CANcoder ID " // TODO: remove pivot @cycIes
                            + CANcoderID
                            + " failed config with error "
                            + response.toString());
        }
    }
}

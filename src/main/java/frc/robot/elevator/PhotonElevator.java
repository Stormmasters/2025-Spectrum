package frc.robot.elevator;

import frc.spectrumLib.Rio;
import lombok.Getter;

public class PhotonElevator extends Elevator {
    public static class PhotonElevatorConfig extends ElevatorConfig {
        /* Elevator constants in rotations */
        @Getter private double maxRotations = 20.5;

        @Getter private double minRotations = 0.3;

        @Getter private final double stationIntake = 9.67;
        @Getter private final double stationExtendedIntake = 9.67;

        @Getter private final double L1Coral = 0;
        @Getter private final double L2Coral = 0;
        @Getter private final double L3Coral = 10.333008;
        @Getter private final double L4Coral = 20.929199;

        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 1500;
        @Getter private final double positionKd = 140;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.06;
        @Getter private final double positionKa = 0.001;
        @Getter private final double positionKg = 12.5;
        @Getter private final double mmCruiseVelocity = 10;
        @Getter private final double mmAcceleration = 50;
        @Getter private final double mmJerk = 0;

        public PhotonElevatorConfig() {
            super();
            setPhoton(true);
            setStationIntake(stationIntake);
            setStationExtendedIntake(stationExtendedIntake);
            setL1Coral(L1Coral);
            setL2Coral(L2Coral);
            setL3Coral(L3Coral);
            setL4Coral(L4Coral);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setFollowerConfigs(new FollowerConfig("ElevatorRear", 41, Rio.CANIVORE, true));
        }
    }

    public PhotonElevator(PhotonElevatorConfig config) {
        super(config);
    }
}

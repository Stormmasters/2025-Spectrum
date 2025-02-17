package frc.robot.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.*;

public class ElevatorRight extends Mechanism {

    public static class ElevatorRightConfig extends Config {
        /* ElevatorRight constants in rotations */
        @Getter private double maxRotations = 110.6304660319; // 29.8;
        @Getter private double minRotations = 0;

        /* ElevatorRight positions in rotations */
        // TODO: Find elevatorRight positions
        @Getter @Setter private double fullExtend = maxRotations * .999;
        @Getter private double home = minRotations;
        @Getter private final double l1 = 10;
        @Getter private final double l2 = 15.412;
        @Getter private final double l3 = 24.4168;
        @Getter private final double l4 = fullExtend;
        @Getter private final double barge = fullExtend;
        @Getter private final double stationIntake = 10.5;
        @Getter private double stationExtendedIntake = 14.5;

        @Getter private double tolerance = 0.95;
        @Getter private double elevatorRightUpHeight = 5;

        /* ElevatorRight config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 0.86; // 20 FOC // 10 Regular
        @Getter private final double positionKv = 0.13; // .12 FOC // .15 regular
        @Getter private double currentLimit = 20;
        @Getter private final double torqueCurrentLimit = 100;

        /* Sim properties */
        @Getter private double kElevatorRightGearing = 3.62722; // 5;
        @Getter private double kCarriageMass = 1;
        @Getter private double kElevatorRightDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double initialX = 0.8;
        @Getter private double initialY = 0.35;
        @Getter private double angle = 90;
        @Getter private double staticLength = 50;
        @Getter private double movingLength = 50;

        public ElevatorRightConfig() {
            super("ElevatorRight", 41, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(700, 900, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }

        /** Use these method to set the config for the mechanism on each robot */
        public void configSupplyCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            configSupplyCurrentLimit(currentLimit, true);
        }
    }

    private ElevatorRightConfig config;

    public ElevatorRight(ElevatorRightConfig config) {
        super(config);
        this.config = config;

        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ElevatorStates.setStates();
    }

    public void setupDefaultCommand() {
        ElevatorStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position Percentage", this::getPositionPercentage, null);
            builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("StatorCurrent", this::getCurrent, null);
            builder.addDoubleProperty("#FullExtend", config::getFullExtend, config::setFullExtend);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the elevatorRight. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("ElevatorRight.holdPosition");
                addRequirements(ElevatorRight.this);
            }

            @Override
            public void initialize() {
                stop();
                holdPosition = getPositionRotations();
            }

            @Override
            public void execute() {
                double currentPosition = getPositionRotations();
                if (Math.abs(holdPosition - currentPosition) <= 5) {
                    setMMPosition(() -> holdPosition);
                } else {
                    stop();
                    DriverStation.reportError(
                            "ElevatorRightHoldPosition tried to go too far away from current position. Current Position: "
                                    + currentPosition
                                    + " || Hold Position: "
                                    + holdPosition,
                            false);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command zeroElevatorRightRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("ElevatorRight.zeroElevatorRightRoutine");
    }

    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> stop())
                .withName("ElevatorRight.waitForElbow")
                .until(
                        () ->
                                (ElevatorStates.getElbowShoulderPos().getAsDouble() < 50.0)
                                        || ElevatorStates.getPosition().getAsDouble()
                                                < rotations.getAsDouble()
                                        || ElevatorStates.getPosition().getAsDouble()
                                                > config.getL2())
                .andThen(
                        run(() -> setMMPosition(rotations))
                                .withName("ElevatorRight.moveToRotations"));
    }
}

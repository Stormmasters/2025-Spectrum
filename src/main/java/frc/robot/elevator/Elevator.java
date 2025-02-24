package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Elevator extends Mechanism {

    public static class ElevatorConfig extends Config {
        /* Elevator constants in rotations */
        @Getter private double maxRotations = 20.5; // TODO: Reset to 21.1;

        @Getter
        private double minRotations = 0.3; // This is to prevent it from driving to zero too hard

        /* Elevator positions in rotations */
        // TODO: Find elevator positions
        @Getter @Setter private double fullExtend = maxRotations * .999;
        @Getter private double home = 0.3;

        @Getter private final double algaeLollipop = 0.3; // TODO: find real value
        @Getter private final double coralLollipop = 0.3; // TODO: find real value

        @Getter private final double clawGroundAlgaeIntake = 0.3; // TODO: find real value
        @Getter private final double clawGroundCoralIntake = 0.3; // TODO: find real value

        @Getter private final double stationIntake = 2.7;
        @Getter private final double stationExtendedIntake = 6.5;

        @Getter private final double handOff = 5.5; // TODO: check if this works

        @Getter private final double l2Algae = 0.3;
        @Getter private final double preL3Algae = 3; // TODO: find this value
        @Getter private final double l3Algae = 12.5;

        @Getter private final double l1Coral = 0.3;
        @Getter private final double preL2Coral = 3; // TODO: find this value
        @Getter private final double l2Coral = 7.15;
        @Getter private final double preL3Coral = 3; // TODO: find this value
        @Getter private final double l3Coral = 17.5;
        @Getter private final double preL4Coral = 3; // TODO: find this value
        @Getter private final double l4Coral = 18.86;

        @Getter private final double barge = 20;

        @Getter private double triggerTolerance = 0.95;
        @Getter private double elevatorIsUpHeight = 5;
        @Getter private double elevatorIsHighHeight = 10;
        @Getter private double initPosition = 0;
        @Getter private double holdMaxSpeedRPM = 100;

        /* Elevator config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 100;
        @Getter private final double positionKd = 6;
        @Getter private final double positionKa = 0.2;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 5;
        @Getter private final double positionKg = 25.3;
        @Getter private final double mmCruiseVelocity = 40;
        @Getter private final double mmAcceleration = 280;
        @Getter private final double mmJerk = 2000;

        @Getter private double currentLimit = 40;
        @Getter private double torqueCurrentLimit = 160;

        /* Sim properties */
        @Getter private double kElevatorGearing = 1.7;
        @Getter private double kCarriageMass = 13.6078;
        @Getter private double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double initialX = 0.8;
        @Getter private double initialY = 0.35;
        @Getter private double angle = 90;
        @Getter private double staticLength = 50;
        @Getter private double movingLength = 50;

        public ElevatorConfig() {
            super("ElevatorFront", 40, Rio.CANIVORE);
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

        /** Use these method to set the config for the mechanism on each robot */
        public void configSupplyCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            configSupplyCurrentLimit(currentLimit, true);
        }
    }

    private ElevatorConfig config;
    @Getter private ElevatorSim sim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

        simulationInit();
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

    private void setInitialPosition() {
        if (isAttached()) {
            motor.setPosition(config.getInitPosition());
            followerMotors[0].setPosition(config.getInitPosition());
        }
    }

    public Command resetToInitialPos() {
        return run(() -> setInitialPosition());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the elevator. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elevator.holdPosition");
                addRequirements(Elevator.this);
            }

            @Override
            public void initialize() {
                holdPosition = getPositionRotations();
                setMMPositionFoc(() -> holdPosition);
            }

            @Override
            public void execute() {
                double currentPosition = getPositionRotations();
                if (Math.abs(holdPosition)
                        < 0.05) { // Added so it doesn't try to hold when all the way down
                    stop();
                } else if (Math.abs(getVelocityRPM()) > config.holdMaxSpeedRPM) {
                    stop(); // Don't hold if moving too fast
                    holdPosition = currentPosition; // Update to a new hold position
                } else if (Math.abs(holdPosition - currentPosition) <= 2) {
                    setMMPositionFoc(() -> holdPosition);
                } else {
                    stop();
                    DriverStation.reportError(
                            "ElevatorHoldPosition tried to go too far away from current position. Current Position: "
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

    public Command zeroElevatorRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Elevator.zeroElevatorRoutine");
    }

    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> stop())
                .withName("Elevator.waitForElbow")
                .until(
                        () ->
                                (ElevatorStates.getElbowShoulderPos().getAsDouble() < 50.0)
                                        || ElevatorStates.getPosition().getAsDouble()
                                                < rotations.getAsDouble()
                                        || ElevatorStates.getPosition().getAsDouble()
                                                > config.getL2Coral())
                .andThen(
                        run(() -> setMMPositionFoc(rotations))
                                .withName("Elevator.moveToRotations"));
    }

    // TODO: remove after testing
    public Command setElevatorMMPositionFOC(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName("Elevator Set MM Position");
    }

    public Command moveToRelativePosition(DoubleSupplier position) {
        return new InstantCommand(
                () -> setMMPositionFoc(() -> getPositionRotations() + position.getAsDouble()));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) { // Only run simulation if it's attached
            sim = new ElevatorSim(motor.getSimState(), RobotSim.leftView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) { // Only run if it's attached
            sim.simulationPeriodic();
        }
    }

    class ElevatorSim extends LinearSim {
        public ElevatorSim(TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
            super(
                    new LinearConfig(
                                    config.initialX,
                                    config.initialY,
                                    config.kElevatorGearing,
                                    config.kElevatorDrumRadiusMeters)
                            .setAngle(config.angle)
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(30.5 + 7),
                    mech,
                    elevatorMotorSim,
                    "1" + config.getName()); // added 1 to the name to create it first
        }
    }
}

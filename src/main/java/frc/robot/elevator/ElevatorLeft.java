package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class ElevatorLeft extends Mechanism {

    public static class ElevatorLeftConfig extends Config {
        /* ElevatorLeft constants in rotations */
        @Getter private double maxRotations = 110.6304660319; // 29.8;
        @Getter private double minRotations = 0;

        /* ElevatorLeft positions in rotations */
        // TODO: Find elevatorLeft positions
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
        @Getter private double elevatorLeftUpHeight = 5;

        /* ElevatorLeft config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 0.86; // 20 FOC // 10 Regular
        @Getter private final double positionKv = 0.13; // .12 FOC // .15 regular
        @Getter private double currentLimit = 20;
        @Getter private final double torqueCurrentLimit = 100;

        /* Sim properties */
        @Getter private double kElevatorLeftGearing = 3.62722; // 5;
        @Getter private double kCarriageMass = 1;
        @Getter private double kElevatorLeftDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double initialX = 0.8;
        @Getter private double initialY = 0.35;
        @Getter private double angle = 90;
        @Getter private double staticLength = 50;
        @Getter private double movingLength = 50;

        public ElevatorLeftConfig() {
            super("ElevatorLeft", 40, Rio.CANIVORE);
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

    private ElevatorLeftConfig config;
    @Getter private ElevatorLeftSim sim;

    public ElevatorLeft(ElevatorLeftConfig config) {
        super(config);
        this.config = config;

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

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the elevatorLeft. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("ElevatorLeft.holdPosition");
                addRequirements(ElevatorLeft.this);
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
                            "ElevatorLeftHoldPosition tried to go too far away from current position. Current Position: "
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

    public Command zeroElevatorLeftRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("ElevatorLeft.zeroElevatorLeftRoutine");
    }

    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> stop())
                .withName("ElevatorLeft.waitForElbow")
                .until(
                        () ->
                                (ElevatorStates.getElbowShoulderPos().getAsDouble() < 50.0)
                                        || ElevatorStates.getPosition().getAsDouble()
                                                < rotations.getAsDouble()
                                        || ElevatorStates.getPosition().getAsDouble()
                                                > config.getL2())
                .andThen(
                        run(() -> setMMPosition(rotations))
                                .withName("ElevatorLeft.moveToRotations"));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) { // Only run simulation if it's attached
            sim = new ElevatorLeftSim(motor.getSimState(), RobotSim.leftView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) { // Only run if it's attached
            sim.simulationPeriodic();
        }
    }

    class ElevatorLeftSim extends LinearSim {
        public ElevatorLeftSim(TalonFXSimState elevatorLeftMotorSim, Mechanism2d mech) {
            super(
                    new LinearConfig(
                                    config.initialX,
                                    config.initialY,
                                    config.kElevatorLeftGearing,
                                    config.kElevatorLeftDrumRadiusMeters)
                            .setAngle(config.angle)
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(30.5),
                    mech,
                    elevatorLeftMotorSim,
                    "1" + config.getName()); // added 1 to the name to create it first
        }
    }
}

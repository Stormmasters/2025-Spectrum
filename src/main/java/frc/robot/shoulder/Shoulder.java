package frc.robot.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import lombok.*;

public class Shoulder extends Mechanism {

    public static class ShoulderConfig extends Config {

        // Positions set as percentage of shoulder
        @Getter private final int initializedPosition = 20;

        /* Shoulder positions in percentage of max rotation || 0 is horizontal */
        // TODO: Find shoulder positions
        @Getter private final double score = 100 - 65;
        @Getter private final double climbHome = 100 - 3;
        @Getter private final double home = 100 - 1;
        @Getter private final double coralIntake = 44;
        @Getter private final double floorIntake = 99;
        @Getter private final double ninetyDegrees = 100 - 39;
        @Getter private final double l1Coral = 63;
        @Getter private final double l2Algae = 90;
        @Getter private final double l3Algae = 90;
        @Getter private final double l2Coral = 90;
        @Getter private final double l3Coral = 90;
        @Getter private final double l4Coral = 57;
        @Getter private final double barge = 57;
        @Getter @Setter private double tuneShoulder = 0;

        /* Shoulder config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double shoulderX = 0.8;
        @Getter private double shoulderY = 0.8;

        @Getter @Setter private double simRatio = 1;

        @Getter private double length = 0.3;

        public ShoulderConfig() {
            super("Shoulder", 42, Rio.RIO_CANBUS); // Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 147000, 161000, 0);
            configGearRatio(1); // 50.43);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(0, 11.57142857);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setSimRatio(15.429);
        }

        public ShoulderConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private ShoulderConfig config;
    private CANcoder m_CANcoder;
    @Getter private ShoulderSim sim;
    CANcoderSimState canCoderSim;

    public Shoulder(ShoulderConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ShoulderStates.setStates();
    }

    public void setupDefaultCommand() {
        ShoulderStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getPositionRotations, null);
            builder.addDoubleProperty(
                    "Position Percent",
                    () -> (getPositionRotations() / config.getMaxRotations()) * 100,
                    null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
            builder.addDoubleProperty(
                    "#Tune Position Percent", config::getTuneShoulder, config::setTuneShoulder);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroShoulderRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Shoulder.zeroShoulderRoutine");
    }

    /** Holds the position of the Shoulder. */
    public Command runHoldShoulder() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Shoulder.holdPosition");
                addRequirements(Shoulder.this);
            }

            @Override
            public void initialize() {
                holdPosition = getPositionRotations();
            }

            @Override
            public void execute() {
                moveToRotations(() -> holdPosition);
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public boolean ShoulderHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new ShoulderSim(motor.getSimState(), RobotSim.leftView);

            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }

    class ShoulderSim extends ArmSim {
        public ShoulderSim(TalonFXSimState shoulderMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.shoulderX,
                                    config.shoulderY,
                                    config.simRatio,
                                    config.length,
                                    -45,
                                    270 - 45,
                                    -45)
                            .setMount(Robot.getElevator().getSim(), true),
                    mech,
                    shoulderMotorSim,
                    "3" + config.getName()); // added 3 to the name to create it third
        }
    }
}

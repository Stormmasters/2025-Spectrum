package frc.robot.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import lombok.*;

public class Wrist extends Mechanism {

    public static class WristConfig extends Config {

        // Positions set as percentage of wrist
        @Getter private final int initializedPosition = 20;

        /* wrist positions in percentage of max rotation || 0 is horizontal */
        // TODO: Find wrist positions
        @Getter private final double score = 100 - 65;
        @Getter private final double home = 100 - 1;
        @Getter private final double intake = 100;
        @Getter private final double manualFeed = 100 - 70;
        @Getter private final double ninetyDegrees = 100 - 39;
        @Getter private final double l1 = 63;
        @Getter private final double l2Algae = 90; // 100 - 94; // (FM20235)
        @Getter private final double l3Algae = 90; // 100 - 75; // (FM20235)
        @Getter private final double l2Coral = 90; // 0; // (FM20235)
        @Getter private final double l3Coral = 90; // 100 - 94; // (FM20235)
        @Getter private final double l4 = 57;
        @Getter private final double barge = 57;
        @Getter @Setter private double tuneWrist = 0;

        /* Wrist config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter @Setter private boolean shortFeed = false;
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double wristX = 0.8;
        @Getter private double wristY = 0.7;
        @Getter @Setter private double simRatio = 172.8; // TODO: Set this to actual wrist ratio
        @Getter private double length = 0.3;

        public WristConfig() {
            super("Wrist", 42, Rio.RIO_CANBUS); // Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 73500, 80500, 0); // 147000, 161000, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(0, 132.0126953); // 66.0063476563); // .96
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }

        public WristConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private WristConfig config;
    private CANcoder m_CANcoder;
    // @Getter private WristSim sim;
    CANcoderSimState canCoderSim;

    public Wrist(WristConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        WristStates.setStates();
    }

    public void setupDefaultCommand() {
        WristStates.setupDefaultCommand();
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
                    "#Tune Position Percent", config::getTuneWrist, config::setTuneWrist);
        }
    }

    public void switchFeedSpot() {
        config.setShortFeed(!(config.isShortFeed()));
        Telemetry.print("Feed spot switched to " + ((config.isShortFeed()) ? " short" : " long"));
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroWristRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Wrist.zeroWristRoutine");
    }

    /** Holds the position of the Wrist. */
    public Command runHoldWrist() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Wrist.holdPosition");
                addRequirements(Wrist.this);
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

    public boolean WristHasError() {
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
            // sim = new WristSim(motor.getSimState(), RobotSim.leftView);

            // // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // if (isAttached()) {
        //     sim.simulationPeriodic();
        //     // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        // }
    }

    // class WristSim extends ArmSim {
    //     public WristSim(TalonFXSimState wristMotorSim, Mechanism2d mech) {
    //         super(
    //                 new ArmConfig(
    //                                 config.wristX,
    //                                 config.wristY,
    //                                 config.simRatio,
    //                                 config.length,
    //                                 225 - Units.rotationsToDegrees(config.getMaxRotations()) -
    // 90,
    //                                 225 - Units.rotationsToDegrees(config.getMinRotations()) -
    // 90,
    //                                 -45 - 90)
    //                         .setMount(Robot.getElevator().getSim(), false),
    //                 mech,
    //                 wristMotorSim,
    //                 "3" + config.getName()); // added 3 to the name to create it third
    //     }
    // }
}

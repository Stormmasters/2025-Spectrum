package frc.robot.elbow;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.elevator.ElevatorStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Elbow extends Mechanism {

    public static class ElbowConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        /* Elbow positions in percentage of max rotation || 0 is vertical up || positions should be towards the front of the robot */

        // TODO: Find elbow positions
        @Getter private final double handAlgae = 99;
        @Getter private final double home = 0;

        @Getter private final double algaeLollipop = 56.7;
        @Getter private final double coralLollipop = 58.3;
        @Getter private final double stationIntake = 14.4;
        @Getter private final double stationExtendedIntake = 24.4;
        @Getter private final double floorIntake = 0;
        @Getter private final double clawGroundAlgaeIntake = 58.3;
        @Getter private final double clawGroundCoralIntake = 58.3;
        @Getter private final double handOff = 100;

        @Getter private final double l2Algae = 33.3;
        @Getter private final double l3Algae = 33.3;

        @Getter private final double l1Coral = 36.1;
        @Getter private final double l2Coral = 30.6;
        @Getter private final double l3Coral = 30.6;
        @Getter private final double l4Coral = 25;

        @Getter private final double barge = 7.2;

        @Getter @Setter private double tuneElbow = 0;
        @Getter @Setter private boolean leftScore = true;

        /* Elbow config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        /* Cancoder config settings */
        @Getter private final double CANcoderGearRatio = 30 / 36;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double elbowX = 0.8; // 1.0;
        @Getter private double elbowY = 0.8;
        @Getter @Setter private double simRatio = 1;
        @Getter private double length = 0.4;
        @Getter private double startingAngle = 180 - 90;

        public ElbowConfig() {
            super("Elbow", 43, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 147000, 161000, 0);
            configGearRatio(102.857);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-25.201172*2, 25.201172*2); // calculated to be 51.4285
            configReverseSoftLimit(-25.201172, true);
            configForwardSoftLimit(25.201172, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            setSimRatio(102.857);
        }

        public ElbowConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    @Getter private ElbowConfig config;
    private SpectrumCANcoder canCoder;
    @Getter private ElbowSim sim;
    CANcoderSimState canCoderSim;

    public Elbow(ElbowConfig config) {
        super(config);
        this.config = config;

        canCoder =
                new SpectrumCANcoder(43, motor, config)
                        .setGearRatio(config.getCANcoderGearRatio())
                        .setOffset(config.getCANcoderOffset())
                        .setAttached(true);

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ElbowStates.setStates();
    }

    public void setupDefaultCommand() {
        ElbowStates.setupDefaultCommand();
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
                    "#Tune Position Percent", config::getTuneElbow, config::setTuneElbow);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroElbowRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            canCoder.getCanCoder().setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Elbow.zeroElbowRoutine");
    }

    /** Holds the position of the Elbow. */
    public Command runHoldElbow() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elbow.holdPosition");
                addRequirements(Elbow.this);
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

    public boolean ElbowHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    public Command moveToPercentage(DoubleSupplier percent) {
        return runHoldElbow()
                .until(() -> ((ElevatorStates.allowedPosition()) || percent.getAsDouble() < 50))
                .andThen(
                        run(() -> setMMPosition(() -> percentToRotations(percent)))
                                .withName(getName() + ".runPosePercentage"));
    }

    public double checkReversed(DoubleSupplier position) {
        if (!config.isReversed()) {
            return position.getAsDouble();
        }

        return position.getAsDouble() * -1;
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new ElbowSim(motor.getSimState(), RobotSim.leftView);

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

    class ElbowSim extends ArmSim {
        public ElbowSim(TalonFXSimState elbowMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.elbowX,
                                    config.elbowY,
                                    config.simRatio,
                                    config.length,
                                    -90,
                                    180 + 90,
                                    config.getStartingAngle())
                            .setColor(new Color8Bit(Color.kAqua))
                            .setMount(Robot.getShoulder().getSim(), true),
                    mech,
                    elbowMotorSim,
                    "3" + config.getName()); // added 3 to the name to create it third
        }
    }
}

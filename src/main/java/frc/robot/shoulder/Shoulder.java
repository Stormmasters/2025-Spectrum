package frc.robot.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Shoulder extends Mechanism {

    public static class ShoulderConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        // Positions set as degrees of rotation || 0 is vertical down
        @Getter private final int initializedPosition = 0;

        /* Shoulder positions in degrees || 0 is vertical down || positions should be towards front of robot */
        // TODO: Find shoulder positions
        @Getter private final double score = 117;
        @Getter private final double climbHome = 180;
        @Getter private final double handAlgae = -144; // TODO: find this value
        @Getter private final double home = 0;

        @Getter private final double algaeLollipop = 0;
        @Getter private final double coralLollipop = -157; // TODO: find this value
        @Getter private final double stationIntake = 0;
        @Getter private final double stationExtendedIntake = 172; // TODO: find this value
        @Getter private final double clawGroundAlgaeIntake = -164; // TODO: find this value
        @Getter private final double clawGroundCoralIntake = -164; // TODO: find this value
        @Getter private final double handOff = 180;

        @Getter private final double l2Algae = 43.07; // TODO: find this value
        @Getter private final double l3Algae = 43.07; // TODO: find this value

        @Getter private final double l1Coral = 14; // TODO: find this value
        @Getter private final double l2Coral = 34;
        @Getter private final double l3Coral = 34;
        @Getter private final double l4Coral = 180;

        @Getter private final double barge = -94.4;
        @Getter @Setter private double tuneShoulder = 0;

        @Getter private final double tolerance = 0.95;

        @Getter private final double offset = -90;
        @Getter private final double initPosition = 0;

        /* Shoulder config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 20;
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

        /* Cancoder config settings */
        @Getter private final double CANcoderGearRatio = 30 / 36;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double shoulderX = 0.8;
        @Getter private double shoulderY = 1.1;

        @Getter @Setter private double simRatio = 1;

        @Getter private double length = 0.3;

        public ShoulderConfig() {
            super("Shoulder", 42, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(102.857);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(-0.75, 0.25);
            configReverseSoftLimit(-0.75, true);
            configForwardSoftLimit(0.25, true);
            configNeutralBrakeMode(true);
            if (Robot.isSimulation()) {
                configCounterClockwise_Positive();
            } else {
                configClockwise_Positive();
            }
            configGravityType(true);
            setSimRatio(102.857);
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
    private SpectrumCANcoder canCoder;
    @Getter private ShoulderSim sim;
    CANcoderSimState canCoderSim;

    public Shoulder(ShoulderConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            canCoder =
                    new SpectrumCANcoder(42, motor, config)
                            .setGearRatio(config.getCANcoderGearRatio())
                            .setOffset(config.getCANcoderOffset())
                            .setAttached(false);
            setIntialPosition();
        }

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
            builder.addDoubleProperty(
                    "Position Degrees", () -> (this.getPositionDegrees() - config.offset), null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
            builder.addDoubleProperty(
                    "#Tune Position Percent", config::getTuneShoulder, config::setTuneShoulder);
        }
    }

    private void setIntialPosition() {
        if (canCoder.isAttached()) {
            motor.setPosition(
                    canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                            * config.getGearRatio());
        } else {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        }
    }

    public Command resetToIntialPos() {
        return run(() -> setIntialPosition());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroShoulderRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            canCoder.getCanCoder().setPosition(0);
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
                addRequirements(Robot.getShoulder());
            }

            @Override
            public void initialize() {
                holdPosition = getPositionRotations();
                stop();
            }

            @Override
            public void execute() {
                if (Math.abs(getVelocityRPM()) > config.holdMaxSpeedRPM) {
                    stop();
                    holdPosition = getPositionRotations();
                } else {
                    setDynMMPositionFoc(
                            () -> holdPosition,
                            () -> config.getMmCruiseVelocity(),
                            () -> config.getMmAcceleration(),
                            () -> 20);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public boolean ShoulderHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    public double checkReversed(DoubleSupplier position) {
        if (!config.isReversed()) {
            return position.getAsDouble();
        }

        return position.getAsDouble() * -1;
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    public Command moveToDegreesAndCheckReversed(DoubleSupplier degrees) {
        return moveToDegrees(() -> checkReversed(degrees));
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
                                    -270,
                                    360 - 90,
                                    -90)
                            .setMount(Robot.getElevator().getSim(), true),
                    mech,
                    shoulderMotorSim,
                    "2" + config.getName()); // added 2 to the name to create it second
        }
    }
}

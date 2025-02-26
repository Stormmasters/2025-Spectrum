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
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class PhotonShoulder extends Mechanism {

    public static class PhotonShoulderConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        // Positions set as percentage of photonShoulder
        @Getter private final int initializedPosition = 20;

        /* PhotonShoulder positions in percentage of max rotation || 0 is vertical down || positions should be towards front of robot */
        // TODO: Find photonShoulder positions
        @Getter private final double stationIntake = 36.03515625;

        @Getter private final double l2Coral = 232.3828125;
        @Getter private final double l3Coral = 233.4;
        @Getter private final double l4Coral = 203.927734375;
        @Getter @Setter private double tunePhotonShoulder = 0;

        @Getter private final double tolerance = 0.95;

        @Getter private final double offset = -90;
        @Getter private final double initPosition = 0;

        /* PhotonShoulder config settings */
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

        /* Cancoder config settings */
        @Getter private final double CANcoderGearRatio = 30 / 36;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double photonShoulderX = 0.8;
        @Getter private double photonShoulderY = 1.1;

        @Getter @Setter private double simRatio = 1;

        @Getter private double length = 0.3;

        // TODO: get correct configs
        public PhotonShoulderConfig() {
            super("PhotonShoulder", 42, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk); // 147000, 161000, 0);
            configGearRatio(102.857);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-torqueCurrentLimit);
            configMinMaxRotations(-0.25, 0.75);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            if (Robot.isSimulation()) {
                configCounterClockwise_Positive();
            } else {
                configCounterClockwise_Positive();
            }
            configGravityType(true);
            setSimRatio(102.857);
        }

        public PhotonShoulderConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private PhotonShoulderConfig config;
    private SpectrumCANcoder canCoder;
    @Getter private PhotonShoulderSim sim;
    CANcoderSimState canCoderSim;

    public PhotonShoulder(PhotonShoulderConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            canCoder =
                    new SpectrumCANcoder(42, motor, config)
                            .setGearRatio(config.getCANcoderGearRatio())
                            .setOffset(config.getCANcoderOffset())
                            .setAttached(false);
            setInitialPosition();
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
            builder.addDoubleProperty("Position Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty(
                    "Position Degrees", () -> (this.getPositionDegrees() - config.offset), null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
            builder.addDoubleProperty(
                    "#Tune Position Percent",
                    config::getTunePhotonShoulder,
                    config::setTunePhotonShoulder);
        }
    }

    private void setInitialPosition() {
        if (canCoder.isAttached()) {
            motor.setPosition(
                    canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                            * config.getGearRatio());
        } else {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        }
    }

    public Command resetToIntialPos() {
        return run(() -> setInitialPosition());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroPhotonShoulderRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            canCoder.getCanCoder().setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("PhotonShoulder.zeroPhotonShoulderRoutine");
    }

    /** Holds the position of the PhotonShoulder. */
    public Command runHoldPhotonShoulder() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("PhotonShoulder.holdPosition");
                addRequirements(PhotonShoulder.this);
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

    public boolean PhotonShoulderHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            // sim = new PhotonShoulderSim(motor.getSimState(), RobotSim.leftView);

            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            // sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }

    class PhotonShoulderSim extends ArmSim {
        public PhotonShoulderSim(TalonFXSimState photonShoulderMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.photonShoulderX,
                                    config.photonShoulderY,
                                    config.simRatio,
                                    config.length,
                                    -270,
                                    360 - 90,
                                    -90)
                            .setMount(Robot.getElevator().getSim(), true),
                    mech,
                    photonShoulderMotorSim,
                    "2" + config.getName()); // added 2 to the name to create it second
        }
    }
}

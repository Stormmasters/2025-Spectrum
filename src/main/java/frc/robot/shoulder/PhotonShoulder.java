package frc.robot.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class PhotonShoulder extends Shoulder {

    public static class PhotonShoulderConfig extends ShoulderConfig {
        // Positions set as percentage of photonShoulder
        @Getter private final int initializedPosition = 20;

        @Getter private final double tolerance = 0.95;

        @Getter private final double offset = -90;
        @Getter private final double initPosition = 0;

        @Getter private final double stationIntake = 36.03515625;

        @Getter private final double l1Coral = 36;
        @Getter private final double l2Coral = 232.3828125;
        @Getter private final double l3Coral = 233.4;
        @Getter private final double l4Coral = 203.927734375;

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
        @Getter private final double CANcoderGearRatio = 30.0 / 36.0;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double photonShoulderX = 0.8;
        @Getter private double photonShoulderY = 1.1;

        @Getter @Setter private double simRatio = 1;

        @Getter private double length = 0.3;

        public PhotonShoulderConfig() {
            super();
            setPhoton(true);
            setInitPosition(initPosition);
            setStationIntake(stationIntake);
            setStationExtendedIntake(stationIntake);
            setL1Coral(l1Coral);
            setL2Coral(l2Coral);
            setL2CoralScore(l2Coral);
            setL3Coral(l3Coral);
            setL3CoralScore(l3Coral);
            setL4Coral(l4Coral);
            setL4CoralScore(l4Coral);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(102.857);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-torqueCurrentLimit);
            configMinMaxRotations(-0.25, 0.75);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configGravityType(true);
            setSimRatio(102.857);
        }

        @Override
        public PhotonShoulderConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private PhotonShoulderConfig config;
    private PhotonShoulderSim sim;

    public PhotonShoulder(PhotonShoulderConfig configInput) {
        super(configInput);
        this.config = configInput;

        if (config.isPhoton()) {
            if (isAttached()) {
                canCoder =
                        new SpectrumCANcoder(42, motor, config)
                                .setGearRatio(config.getCANcoderGearRatio())
                                .setOffset(config.getCANcoderOffset())
                                .setAttached(false);
                setInitialPhotonPosition();
            }

            simulationInit();
            telemetryInit();
            Telemetry.print(getName() + " Subsystem Initialized");
        }
    }

    @Override
    public void periodic() {}

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
        }
    }

    void setInitialPhotonPosition() {
        if (canCoder.isAttached()) {
            motor.setPosition(
                    canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                            * config.getGearRatio());
        } else {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        }
    }

    @Override
    public Command resetToIntialPos() {
        return run(this::setInitialPhotonPosition);
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

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    @Override
    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    @Override
    void simulationInit() {
        if (isAttached()) {
            sim = new PhotonShoulderSim(motor.getSimState(), RobotSim.leftView);

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

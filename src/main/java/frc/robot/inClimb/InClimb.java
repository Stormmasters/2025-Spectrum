package frc.robot.inClimb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumServo;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class InClimb extends Mechanism {

    public static class InClimbConfig extends Config {

        @Getter private final double maxRotations = 0.4; // TODO: find max rotations
        @Getter private final double minRotations = -0.085;
        /* InClimb positions in degrees || 0 is horizontal */
        @Getter private final double home = 90;
        @Getter private final double intake = 0;
        @Getter private final double algaeFloorIntake = 30;
        @Getter @Setter private double tuneInClimb = 0;
        @Getter private final double prepClimber = -10;
        @Getter private final double finishClimb = 100;
        @Getter private final double coralFloorIntake = -10;
        @Getter private final double processorScore = 60;
        @Getter private final double latchOpen = 1;
        @Getter private final double latchClosed = 0;

        @Getter private final double offsetConstant = -90;

        /* InClimb config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 60;
        @Getter private final double torqueCurrentLimit = 180;
        @Getter private final double positionKp = 190;
        @Getter private final double positionKd = 40;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.3;
        @Getter private final double positionKa = 0.001;
        @Getter private final double positionKg = 2.9;
        @Getter private final double mmCruiseVelocity = 4;
        @Getter private final double mmAcceleration = 40;
        @Getter private final double mmJerk = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double InClimbX = 0.95; // 1.0;
        @Getter private double InClimbY = 0.55;

        @Getter @Setter
        private double simRatio =
                99.5555555555; // TODO: Set to number of rotations per mech revolution

        @Getter private double length = 0.4;

        public InClimbConfig() {
            super("InClimbTop", 55, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(99.5555555555);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(getMinRotations(), getMaxRotations());
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(
                    getMaxRotations(), true); // TODO: increase soft limit (used .35 in testing)
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configGravityType(true);
            setSimRatio(simRatio);
            setFollowerConfigs(new FollowerConfig("InClimbBottom", 56, Rio.CANIVORE, false));
        }

        public InClimbConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private InClimbConfig config;
    private SpectrumServo latchServo = new SpectrumServo(9);
    @Getter private InClimbSim sim;

    public InClimb(InClimbConfig config) {
        super(config);
        this.config = config;

        setIntialPosition();
        setLatchOpen();

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        InClimbStates.setStates();
    }

    public void setupDefaultCommand() {
        InClimbStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position Degrees", () -> (this.getPositionDegrees()), null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
            builder.addDoubleProperty(
                    "#Tune Position Percent", config::getTuneInClimb, config::setTuneInClimb);
        }
    }

    private void setIntialPosition() {
        if (config.isAttached()) {
            motor.setPosition(0.25); // TODO: Remove once mechanism does this for everything
            followerMotors[0].setPosition(0.25);
        }
    }

    private void setLatchOpen() {
        latchServo.set(config.getLatchOpen());
    }

    private void setLatchClosed() {
        latchServo.set(config.getLatchClosed());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command resetToIntialPos() {
        return runOnce(() -> setIntialPosition())
                .ignoringDisable(true)
                .withName("InClimb.ResetToInitialPos");
    }

    public Command runHoldInClimb() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Pivot.holdPosition");
                addRequirements(InClimb.this);
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
        };
    }

    public Command zeroInClimbRoutine() {
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

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(degrees).withName(getName() + ".runPoseDegrees");
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffsetConstant());
    }

    // TODO: remove after testing
    public Command setInClimbMMPositionFOC(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName("InClimb Set MM Position");
    }

    public Command openLatch() {
        return new RunCommand(() -> setLatchOpen(), latchServo).withName("InClimbLatch.openLatch");
    }

    public Command closeLatch() {
        return new RunCommand(() -> setLatchClosed(), latchServo)
                .withName("InClimbLatch.closeLatch");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new InClimbSim(motor.getSimState(), RobotSim.frontView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class InClimbSim extends ArmSim {
        public InClimbSim(TalonFXSimState InClimbMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.InClimbX,
                                    config.InClimbY,
                                    config.simRatio,
                                    config.length,
                                    -30,
                                    180,
                                    180)
                            .setColor(new Color8Bit(Color.kBrown)),
                    mech,
                    InClimbMotorSim,
                    config.getName());
        }
    }
}

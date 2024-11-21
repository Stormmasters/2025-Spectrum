package frc.robot.amptrap;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import lombok.Getter;

public class AmpTrap extends Mechanism {

    public static class AmpTrapConfig extends Config {

        /* Revolutions per min AmpTrap Output */
        @Getter private final double maxSpeed = 5000;
        @Getter private final double intake = 3000;
        @Getter private final double feed = 500;
        @Getter private final double amp = 4500;
        @Getter private final double score = 4500;
        @Getter private final double eject = -3000;

        /* Percentage AmpTrap Output */
        @Getter private final double slowIntakePercentage = 0.1;

        /* AmpTrap config values */
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = 0.156152;
        @Getter private final double velocityKv = 0.12;
        @Getter private final double velocityKs = 0.24;

        @Getter private final double hasNoteDistance = 300;
        @Getter private final double topHasNoteDistance = 150;

        /* Sim Configs */
        @Getter private double ampTrapX = 0.25;
        @Getter private double ampTrapY = 0.4;
        @Getter private double wheelDiameter = 5.0;

        public AmpTrapConfig() {
            super("AmpTrap", 51, RobotConfig.RIO_CANBUS);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configMotionMagic(51, 205, 0);
        }
    }

    private AmpTrapConfig config;
    private RollerSim sim;
    // TODO: add lasercans

    protected SpectrumState hasNote = new SpectrumState("AmpTrapHasNote");

    public AmpTrap(AmpTrapConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        AmpTrapStates.setStates();
    }

    public void setupDefaultCommand() {
        AmpTrapStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getPositionRotations, null);
            builder.addDoubleProperty("Velocity RPS", this::getVelocityRPS, null);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------

    public void simulationInit() {
        if (isAttached()) {
            sim = new AmpTrapSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic(); // config.ampTrapX, config.ampTrapY);
        }
    }

    class AmpTrapSim extends RollerSim {
        public AmpTrapSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameter)
                            .setPosition(config.ampTrapX, config.ampTrapY)
                            .setMount(Robot.getFakeElevator().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}

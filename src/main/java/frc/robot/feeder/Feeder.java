package frc.robot.feeder;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Feeder extends Mechanism {

    public static class FeederConfig extends Config {

        /* Revolutions per min Feeder Output */
        @Getter private double maxSpeed = 5000;
        @Getter private double intake = 200;
        @Getter private double eject = -3000;
        @Getter private double score = 1000;
        @Getter private double slowFeed = 500;
        @Getter private double slowEject = -500;

        @Getter
        private double feedToAmp = -3500; // Needs to be greater than or equal to amp roller speed

        @Getter private double launchEject = 1000;
        @Getter private double autoFeed = 3000;
        @Getter private double ejectFromIntake = 3000;
        @Getter private double manualSource = -2000;

        /* Rotations config */
        @Getter private double addedFeedRotations = 4;

        /* Percentage Feeder Output */
        @Getter private double slowFeederPercentage = 0.15;

        /* Feeder config values */
        @Getter private double currentLimit = 30;
        @Getter private double torqueCurrentLimit = 100;
        @Getter private double velocityKp = 0.156152;
        @Getter private double velocityKv = 0.12;
        @Getter private double velocityKs = 0.24;
        @Getter private double positionKp = 2;
        @Getter private double positionKv = 0.013;

        /* Sim Configs */
        @Getter private double feederX = 0.475;
        @Getter private double feederY = 0.075;
        @Getter private double wheelDiameter = 4.0;

        public FeederConfig() {
            super("Feeder", 40, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0); // velocity
            configFeedForwardGains(velocityKs, velocityKv, 0, 0); // velocity
            configPIDGains(1, positionKp, 0, 0);
            configFeedForwardGains(1, 0, positionKv, 0, 0);
            configGearRatio(12 / 30);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configMotionMagic(51, 205, 0);
        }
    }

    private FeederConfig config;
    private RollerSim sim;

    public Feeder(FeederConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        FeederStates.setStates();
    }

    public void setupDefaultCommand() {
        FeederStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getMotorPositionRotations, null);
            builder.addDoubleProperty("Velocity RPS", this::getMotorVelocityRPS, null);
        }
    }

    // TODO: add lasercan

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /**
     * Runs the feeder a certain amount of revolutions forward from it's current position.
     *
     * @param revolutions position in revolutions
     */
    public Command runAddPosition(DoubleSupplier revolutions) {
        return runOnce(this::tareMotor).andThen(run(() -> setMMPosition(revolutions, 1)));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------

    public void simulationInit() {
        if (isAttached()) {
            sim = new FeederSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic(config.feederX, config.feederY);
        }
    }

    class FeederSim extends RollerSim {
        public FeederSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(new RollerConfig(config.wheelDiameter), mech, rollerMotorSim, config.getName());
        }
    }
}

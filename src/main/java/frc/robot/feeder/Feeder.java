package frc.robot.feeder;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConfig;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
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
        @Getter private double threshold = 40;
        @Getter private double velocityKp = 0.156152;
        @Getter private double velocityKv = 0.12;
        @Getter private double velocityKs = 0.24;
        @Getter private double positionKp = 2;
        @Getter private double positionKv = 0.013;

        /* Sim configs */

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

    public Feeder(FeederConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void bindTriggers() {
        FeederStates.bindTriggers();
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
            builder.addDoubleProperty("Position", this::getMotorPosition, null);
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

    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}

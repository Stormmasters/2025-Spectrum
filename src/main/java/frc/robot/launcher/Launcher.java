package frc.robot.launcher;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {
        @Getter private double maxVelocityRpm = 5600;
        @Getter private double defaultLaunchRPM = 4000;
        @Getter private double spitRpm = 1000;
        @Getter private double subwooferRPM = 4500;
        @Getter private double ejectRPM = -4000;

        /* LeftLauncher config values */
        @Getter private double currentLimit = 60;
        @Getter private double torqueCurrentLimit = 300;
        @Getter private double velocityKp = 4;
        @Getter private double velocityKv = 0.2;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double launcherX = 0.95;
        @Getter private double launcherY = 0.1;
        @Getter private double wheelDiameterIn = 2;

        @Getter
        private final InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

        public LauncherConfig() {
            super("LeftLauncher", 42, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0.0, 0.0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configMotionMagic(51, 205, 0);
            setFollowerConfigs(new FollowerConfig("RightLauncher", 43, Rio.CANIVORE, true));

            distanceMap.put(0.0, 4500.0);
            distanceMap.put(4.1, 4500.0);
            distanceMap.put(4.11, 5000.0);
            distanceMap.put(5.9, 5000.0);
        }
    }

    private LauncherConfig config;
    private RollerSim sim;

    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;
        simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    public void setupStates() {
        LauncherStates.setStates();
    }

    public void setupDefaultCommand() {
        LauncherStates.setupDefaultCommand();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty("VelocityRPM", this::getVelocityRPM, null);
            builder.addDoubleProperty("StatorCurrent", this::getCurrent, null);
        }
    }

    protected double getRPMfromDistance(DoubleSupplier distance) {
        return config.getDistanceMap().get(distance.getAsDouble());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
            sim = new LauncherSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class LauncherSim extends RollerSim {
        public LauncherSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameterIn)
                            .setPosition(config.launcherX, config.launcherY)
                            .setMount(Robot.getPivot().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}

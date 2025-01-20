package frc.robot.coralIntake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import lombok.Getter;

public class CoralIntake extends Mechanism {

    public static class CoralIntakeConfig extends Config {

        /* Revolutions per min Intake Output */
        @Getter private double maxSpeed = 5000;
        @Getter private double intake = -5000;
        @Getter private double eject = 2000;
        @Getter private double slowIntake = -1000;

        /* Percentage Intake Output */
        @Getter private double slowIntakePercentage = 0.06;

        /* Intake config values */
        @Getter private double currentLimit = 40;
        @Getter private double torqueCurrentLimit = 120;
        @Getter private double velocityKp = 12; // 0.156152;
        @Getter private double velocityKv = 0.2; // 0.12;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double intakeX = 0.1; // 0.5; // relative to elbow at 0 degrees
        @Getter private double intakeY = 0.5; // relative to elbow at 0 degrees
        @Getter private double wheelDiameter = 5.0;

        public CoralIntakeConfig() {
            super("CoralIntake", 5, Rio.RIO_CANBUS);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12.0 / 30.0);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(120, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // might be different on actual robot
            configMotionMagic(51, 205, 0);
        }
    }

    private CoralIntakeConfig config;
    private CoralIntakeSim sim;

    public CoralIntake(CoralIntakeConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        CoralIntakeStates.setStates();
    }

    public void setupDefaultCommand() {
        CoralIntakeStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
            builder.addDoubleProperty("StatorCurrent", this::getCurrent, null);
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
            // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
            sim = new CoralIntakeSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class CoralIntakeSim extends RollerSim {
        public CoralIntakeSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameter)
                            .setPosition(config.intakeX, config.intakeY)
                            .setMount(Robot.getElbow().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}

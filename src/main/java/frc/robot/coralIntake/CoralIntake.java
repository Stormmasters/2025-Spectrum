package frc.robot.coralIntake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
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

        // Algae Voltages and Current
        @Getter private double algaeIntakeVoltage = -8.0;
        @Getter private double algaeIntakeSupplyCurrent = 15.0;
        @Getter private double algaeIntakeTorqueCurrent = 90.0;

        @Getter private double algaeScoreVoltage = 12.0;
        @Getter private double algaeScoreSupplyCurrent = 30;
        @Getter private double algaeScoreTorqueCurrent = 180;

        // Coral Voltages and Current
        @Getter private double coralIntakeVoltage = 9.0;
        @Getter private double coralIntakeSupplyCurrent = 12.0;
        @Getter private double coralIntakeTorqueCurrent = 27.0;

        @Getter private double coralScoreVoltage = -2.0;
        @Getter private double coralScoreSupplyCurrent = 12;
        @Getter private double coralScoreTorqueCurrent = 27;

        /* Revolutions per min Intake Output */
        @Getter private double maxSpeed = 5000;
        @Getter private double intake = 5000;
        @Getter private double eject = -2000;
        @Getter private double slowEject = 100;
        @Getter private double slowIntake = -1000;
        @Getter private double barge = -100;

        /* Percentage Intake Output */
        @Getter private double slowIntakePercentage = 0.06;

        /* Intake config values */
        @Getter private double currentLimit = 15;
        @Getter private double torqueCurrentLimit = 100;
        @Getter private double velocityKp = 12; // 0.156152;
        @Getter private double velocityKv = 0.2; // 0.12;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double intakeX = 0.8; // relative to elbow at 0 degrees
        @Getter private double intakeY = 1.3; // relative to elbow at 0 degrees
        @Getter private double wheelDiameter = 5.0;

        public CoralIntakeConfig() {
            super("CoralIntake", 5, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(120, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive(); // might be different on actual robot
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

    // TODO: check if these actually need to be two separate methods

    public boolean hasIntakeCoral() {
        double motorOutput =
                Robot.getCoralIntake()
                        .getVelocityRPM(); // might be better to check with motor voltage
        // return (Math.abs(motorOutput) < 120);
        return motorOutput < 120; // got rid of abs because motor velocity might drop below 0
    }

    public boolean hasIntakeAlgae() {
        double motorOutput = Robot.getCoralIntake().getVelocityRPM();
        return (Math.abs(motorOutput) < 120);
    }

    public Command algaeIntake() {
        return runVoltage(() -> config.algaeIntakeVoltage)
                .alongWith(
                        setCurrentLimits(
                                () -> config.algaeIntakeSupplyCurrent,
                                () -> config.algaeIntakeTorqueCurrent));
    }

    public Command algaeScore() {
        return runVoltage(() -> config.algaeScoreVoltage)
                .alongWith(
                        setCurrentLimits(
                                () -> config.algaeScoreSupplyCurrent,
                                () -> config.algaeScoreTorqueCurrent));
    }

    public Command coralIntake() {
        return runVoltage(() -> config.coralIntakeVoltage)
                .alongWith(
                        setCurrentLimits(
                                () -> config.coralIntakeSupplyCurrent,
                                () -> config.coralIntakeTorqueCurrent));
    }

    public Command coralScore() {
        return runVoltage(() -> config.coralScoreVoltage)
                .alongWith(
                        setCurrentLimits(
                                () -> config.coralScoreSupplyCurrent,
                                () -> config.coralScoreTorqueCurrent));
    }

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
        public CoralIntakeSim(Mechanism2d mech, TalonFXSimState coralRollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameter)
                            .setPosition(config.intakeX, config.intakeY)
                            .setMount(Robot.getElbow().getSim()),
                    mech,
                    coralRollerMotorSim,
                    config.getName());
        }
    }
}

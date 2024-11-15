package frc.robot.intake;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotConfig;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Intake extends Mechanism {
    public static class IntakeConfig extends Config {

        /* Revolutions per min Intake Output */
        @Getter private double maxSpeed = 5000;
        @Getter private double intake = 2000;
        @Getter private double eject = -2000;
        @Getter private double slowIntake = 1000;

        /* Percentage Intake Output */
        @Getter private double slowIntakePercentage = 0.06;

        /* Intake config values */
        @Getter private double currentLimit = 30;
        @Getter private double torqueCurrentLimit = 120;
        @Getter private double threshold = 40;
        @Getter private double velocityKp = 12; // 0.156152;
        @Getter private double velocityKv = 0.2; // 0.12;
        @Getter private double velocityKs = 14;

        public IntakeConfig() {
            super("Intake", 5, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configStatorCurrentLimit(30, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            configMotionMagic(51, 205, 0);
        }
    }

    private IntakeConfig config;

    public Intake(IntakeConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        IntakeStates.setStates();
    }

    public void setupDefaultCommand() {
        IntakeStates.setupDefaultCommand();
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

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command runManualOutput(DoubleSupplier percentSupplier) {
        return run(() -> setPercentOutput(percentSupplier)).withName("Intake.runManualOutput");
    }

    public Command intakeWithoutCurrentLimit() {
        return new FunctionalCommand(
                        () ->
                                toggleTorqueCurrentLimit(
                                        config::getTorqueCurrentLimit,
                                        false), // init -- turn off current limits
                        () ->
                                setVelocityTorqueCurrentFOC(
                                        config::getIntake), // execute -- run at intake velocity
                        (b) -> {
                            toggleTorqueCurrentLimit(
                                    config::getTorqueCurrentLimit,
                                    true); // end -- reenable current limits
                        },
                        () -> false, // is finished
                        this) // requirement
                .withName("Intake.intakeWithoutCurrentLimit");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------

    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}

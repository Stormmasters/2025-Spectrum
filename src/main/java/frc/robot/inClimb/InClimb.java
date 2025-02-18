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
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class InClimb extends Mechanism {

    public static class InClimbConfig extends Config {

        @Getter private final double maxRotations = .3; // TODO: find max rotations
        @Getter private final double minRotations = -0.1;

        /* InClimb positions in percentage of max rotation || 0 is horizontal */
        @Getter private final double home = 0;
        @Getter private final double intake = 35.5;
        @Getter private final double algaeFloorIntake = 70;
        @Getter @Setter private double tuneInClimb = 0;
        @Getter private final double prepClimber = 180;
        @Getter private final double finishClimb = 90;
        @Getter private final double coralFloorIntake = 95;
        @Getter private final double processorScore = 65;

        @Getter private final double offsetConstant = -90;

        /* InClimb config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter private final double currentLimit = 20;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double InClimbX = 0.95; // 1.0;
        @Getter private double InClimbY = 0.55;

        @Getter @Setter
        private double simRatio = 1; // TODO: Set to number of rotations per mech revolution

        @Getter private double length = 0.4;

        public InClimbConfig() {
            super("InClimbTop", 55, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 147000, 161000, 0);
            configGearRatio(99.5555555555);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(0, 7.714285714); // TODO: find minmax rotations and offset
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setSimRatio(14);
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
    @Getter private InClimbSim sim;

    public InClimb(InClimbConfig config) {
        super(config);
        this.config = config;
        motor.setPosition(0.25); // TODO: Remove once mechanism does this for everything
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
            builder.addDoubleProperty("Position", this::getPositionRotations, null);
            builder.addDoubleProperty(
                    "Position Percent",
                    () -> (getPositionRotations() / config.getMaxRotations()) * 100,
                    null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
            builder.addDoubleProperty(
                    "#Tune Position Percent", config::getTuneInClimb, config::setTuneInClimb);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

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
            }

            @Override
            public void execute() {
                moveToRotations(() -> holdPosition);
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
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffsetConstant());
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

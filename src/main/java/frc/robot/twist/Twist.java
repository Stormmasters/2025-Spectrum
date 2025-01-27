package frc.robot.twist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.Mount;
import frc.spectrumLib.sim.Mount.MountType;
import frc.spectrumLib.sim.Mountable;
import lombok.*;

public class Twist extends Mechanism {

    public static class TwistConfig extends Config {

        // Positions set as percentage of Twist
        @Getter private final int initializedPosition = 20;

        /* twist positions in percentage of max rotation || 0 is horizontal */
        // TODO: Find twist positions
        @Getter private final double score = 100 - 65;
        @Getter private final double home = 100 - 1;
        @Getter private final double intake = 100;
        @Getter private final double ninetyDegrees = 100 - 39;
        @Getter private final double l1 = 63;
        @Getter private final double l2Algae = 90;
        @Getter private final double l3Algae = 90;
        @Getter private final double l2Coral = 90;
        @Getter private final double l3Coral = 90;
        @Getter private final double l4 = 57;
        @Getter private final double barge = 57;
        @Getter @Setter private double tuneTwist = 0;

        /* Twist config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double twistX = 0.7;
        @Getter private double twistY = 1.2;
        @Getter @Setter private double simRatio = 15.429; // TODO: Set this to actual twist ratio
        @Getter private double length = 0.25;

        public TwistConfig() {
            super("Twist", 44, Rio.RIO_CANBUS); // Rio.CANIVORE); // TODO: update ID
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 73500, 80500, 0); // 147000, 161000, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-7.714285714, 7.714285714);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }

        public TwistConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private TwistConfig config;
    private CANcoder m_CANcoder;
    @Getter private TwistSim sim;
    CANcoderSimState canCoderSim;

    public Twist(TwistConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        TwistStates.setStates();
    }

    public void setupDefaultCommand() {
        TwistStates.setupDefaultCommand();
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
                    "#Tune Position Percent", config::getTuneTwist, config::setTuneTwist);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroTwistRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Twist.zeroTwistRoutine");
    }

    /** Holds the position of the Twist. */
    public Command runHoldTwist() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Twist.holdPosition");
                addRequirements(Twist.this);
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

    public boolean TwistHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new TwistSim(motor.getSimState(), RobotSim.leftView);

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

    class TwistSim implements Mountable {
        private SingleJointedArmSim armSim;
        private TalonFXSimState twistMotorSim;
        MechanismRoot2d root;
        MechanismLigament2d leftBase;
        MechanismLigament2d leftProng;
        MechanismLigament2d rightBase;
        MechanismLigament2d rightProng;
        Color8Bit topColor = new Color8Bit(Color.kBlue);
        Color8Bit bottomColor = new Color8Bit(Color.kAqua);
        Mount mount;
        double initMountX;
        double initMountY;
        double initMountAngle;

        public TwistSim(TalonFXSimState twistMotorSim, Mechanism2d mech) {
            armSim =
                    new SingleJointedArmSim(
                            DCMotor.getKrakenX60Foc(config.getNumMotors()),
                            config.getSimRatio(),
                            1.2,
                            config.getLength(),
                            Math.toRadians(-180),
                            Math.toRadians(180),
                            false, // Simulate gravity (change back to true)
                            0);
            this.twistMotorSim = twistMotorSim;

            root = mech.getRoot("Claw pivot", config.getTwistX(), config.getTwistY());
            leftBase =
                    root.append(
                            new MechanismLigament2d(
                                    "3LeftTwistBase", config.getLength(), -90.0, 4.0, bottomColor));
            rightBase =
                    root.append(
                            new MechanismLigament2d(
                                    "2RightTwistBase", config.getLength(), 90.0, 4.0, topColor));
            leftProng =
                    leftBase.append(
                            new MechanismLigament2d(
                                    "3LeftTwistProng", 0.15, 90, 10.0, bottomColor));
            rightProng =
                    rightBase.append(
                            new MechanismLigament2d("2RightTwistProng", 0.15, -90, 4.0, topColor));
            mount = Robot.getElbow().getSim();
            initMountX = Robot.getElbow().getConfig().getElbowX();
            initMountY = Robot.getElbow().getConfig().getElbowY();
            initMountAngle = Robot.getElbow().getConfig().getStartingAngle();
        }

        public void simulationPeriodic() {
            armSim.setInput(twistMotorSim.getMotorVoltage());
            armSim.update(TimedRobot.kDefaultPeriod);
            twistMotorSim.setRawRotorPosition(
                    (Units.radiansToRotations(armSim.getAngleRads() - 0)) * config.getSimRatio());
            twistMotorSim.setRotorVelocity(
                    Units.radiansToRotations(armSim.getVelocityRadPerSec()) * config.getSimRatio());

            /* updates position and angle based on elbow */
            root.setPosition(
                    getUpdatedX(
                            MountType.ARM,
                            config.getTwistX(),
                            config.getTwistY(),
                            initMountX,
                            initMountY,
                            initMountAngle,
                            mount.getMountX(),
                            mount.getMountY(),
                            mount.getDisplacementX(),
                            mount.getDisplacementY(),
                            mount.getAngle()),
                    getUpdatedY(
                            MountType.ARM,
                            config.getTwistX(),
                            config.getTwistY(),
                            initMountX,
                            initMountY,
                            initMountAngle,
                            mount.getMountX(),
                            mount.getMountY(),
                            mount.getDisplacementX(),
                            mount.getDisplacementY(),
                            mount.getAngle()));
            rightBase.setAngle(90 + Math.toDegrees(mount.getAngle()));
            leftBase.setAngle(-90 + Math.toDegrees(mount.getAngle()));

            /* changes which side is closest to the viewer */
            if (getPositionPercentage() > 0) {
                leftBase.setColor(topColor);
                leftProng.setColor(topColor);
                rightBase.setColor(bottomColor);
                rightProng.setColor(bottomColor);
                leftProng.setLineWeight(10.0);
                rightProng.setLineWeight(4.0);
                leftBase.setLength(
                        config.getLength() * ((Math.abs(getPositionPercentage()) - 50) / 50));
                rightBase.setLength(
                        config.getLength() * ((Math.abs(getPositionPercentage()) - 50) / 50));
            } else {
                leftBase.setColor(bottomColor);
                leftProng.setColor(bottomColor);
                rightBase.setColor(topColor);
                rightProng.setColor(topColor);
                leftProng.setLineWeight(4.0);
                rightProng.setLineWeight(10.0);
                leftBase.setLength(
                        -1 * config.getLength() * ((Math.abs(getPositionPercentage()) - 50) / 50));
                rightBase.setLength(
                        -1 * config.getLength() * ((Math.abs(getPositionPercentage()) - 50) / 50));
            }
        }
    }
}

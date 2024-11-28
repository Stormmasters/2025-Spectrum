package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.amptrap.AmpTrap;
import frc.robot.amptrap.AmpTrap.AmpTrapConfig;
import frc.robot.auton.Auton;
import frc.robot.climber.Climber;
import frc.robot.climber.Climber.ClimberConfig;
import frc.robot.configs.FM2024;
import frc.robot.configs.PM2024;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.feeder.Feeder;
import frc.robot.feeder.Feeder.FeederConfig;
import frc.robot.intake.Intake;
import frc.robot.intake.Intake.IntakeConfig;
import frc.robot.launcher.Launcher;
import frc.robot.launcher.Launcher.LauncherConfig;
import frc.robot.leds.LedFull;
import frc.robot.leds.LedFull.LedFullConfig;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.pivot.Pivot;
import frc.robot.pivot.Pivot.PivotConfig;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConfig;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.CrashTracker;
import lombok.Getter;

public class Robot extends SpectrumRobot {
    @Getter private static RobotSim robotSim;
    @Getter private static Config config;

    public enum RobotFault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    public static class Config {
        public SwerveConfig swerve = new SwerveConfig();
        public IntakeConfig intake = new IntakeConfig();
        public FeederConfig feeder = new FeederConfig();
        public ElevatorConfig elevator = new ElevatorConfig();
        public AmpTrapConfig ampTrap = new AmpTrapConfig();
        public PivotConfig pivot = new PivotConfig();
        public LauncherConfig launcher = new LauncherConfig();
        public ClimberConfig climber = new ClimberConfig();
        public LedFullConfig leds = new LedFullConfig();
        public PilotConfig pilot = new PilotConfig();
        public OperatorConfig operator = new OperatorConfig();
    }

    @Getter private static Swerve swerve;
    @Getter private static AmpTrap ampTrap;
    @Getter private static Climber climber;
    @Getter private static Elevator elevator;
    @Getter private static Feeder feeder;
    @Getter private static Intake intake;
    @Getter private static Launcher launcher;
    @Getter private static LedFull leds;
    @Getter private static Operator operator;
    @Getter private static Pilot pilot;
    @Getter private static Pivot pivot;
    @Getter private static VisionSystem visionSystem;
    @Getter private static Auton auton;

    public Robot() {
        super();
        Telemetry.start(true, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");
            robotSim = new RobotSim();

            /** Set up the config */
            switch (Rio.id) {
                case FM_2024:
                    config = new FM2024();
                    break;
                case PM_2024:
                    config = new PM2024();
                    break;
                default: // SIM and UNKNOWN
                    config = new FM2024();
                    break;
            }

            /**
             * Initialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are also subsystems.
             */
            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs

            leds = new LedFull(config.leds);
            operator = new Operator(config.operator);
            pilot = new Pilot(config.pilot);
            swerve = new Swerve(config.swerve);
            Timer.delay(canInitDelay);
            elevator = new Elevator(config.elevator);
            Timer.delay(canInitDelay);
            pivot = new Pivot(config.pivot);
            Timer.delay(canInitDelay);
            ampTrap = new AmpTrap(config.ampTrap);
            Timer.delay(canInitDelay);
            climber = new Climber(config.climber);
            Timer.delay(canInitDelay);
            feeder = new Feeder(config.feeder);
            Timer.delay(canInitDelay);
            intake = new Intake(config.intake);
            Timer.delay(canInitDelay);
            launcher = new Launcher(config.launcher);
            auton = new Auton();
            visionSystem = new VisionSystem(swerve::getRobotPose);

            // Setup Default Commands for all subsystems
            setupDefaultCommands();

            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode This method
     * should be called when each mode is initialized
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        pilot.resetConfig();
        operator.resetConfig();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
    }

    public void clearCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
    }

    @Override // Deprecated
    public void robotInit() {}

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    @Override
    public void robotPeriodic() {
        try {
            /**
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();

            SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        Telemetry.print("### Disabled Init Starting ### ");
        resetCommandsAndButtons();

        Telemetry.print("### Disabled Init Complete ### ");
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        RobotStates.coastMode.setFalse(); // Ensure motors are in brake mode
        Telemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    @Override
    public void autonomousInit() {
        try {
            Telemetry.print("@@@ Auton Init Starting @@@ ");
            clearCommandsAndButtons();

            auton.init();

            Telemetry.print("@@@ Auton Init Complete @@@ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        auton.exit();
        Telemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            Telemetry.print("!!! Teleop Init Starting !!! ");
            resetCommandsAndButtons();

            Telemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        Telemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    @Override
    public void testInit() {
        try {

            Telemetry.print("~~~ Test Init Starting ~~~ ");
            resetCommandsAndButtons();

            Telemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        Telemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    @Override
    public void simulationInit() {
        Telemetry.print("$$$ Simulation Init Starting $$$ ");

        Telemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    @Override
    public void simulationPeriodic() {}
}

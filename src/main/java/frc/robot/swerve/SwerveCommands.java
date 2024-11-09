package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotConfig.RobotType;
import frc.robot.pilot.Pilot;
import java.util.function.DoubleSupplier;

public class SwerveCommands {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();

    protected static void setupDefaultCommand() {
        if (Robot.getRobotConfig().getRobotType() == RobotType.PM) {
            // Use this to set a different command based on robotType
            // Robot.swerve.setDefaultCommand(PhotonPilotCommands.pilotDrive());
            // return;
        }
        swerve.setDefaultCommand(pilotDrive());
        // .withTimeout(0.5)
        // .andThen(headingLockDrive())
        // .ignoringDisable(true)
        // .withName("SwerveCommands.default"));
    }

    protected static void bindTriggers() {
        pilot.getStickSteer().whileTrue(stickSteerDrive());

        pilot.getUpReorient().onTrue(reorientForward());
        pilot.getLeftReorient().onTrue(reorientLeft());
        pilot.getDownReorient().onTrue(reorientBack());
        pilot.getRightReorient().onTrue(reorientRight());
    }

    /**
     * ************************************************************************* Pilot Commands
     * ************************************************************************
     */
    /**
     * Drive the robot using left stick and control orientation using the right stick Only Cardinal
     * directions are allowed
     *
     * @return
     */
    protected static Command stickSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }

    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving */
    protected static Command xBrake() {
        return swerve.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
                .withName("Swerve.Xbrake");
    }

    /**
     * ************************************************************************* Helper Commands
     * ************************************************************************
     */
    protected static Command resetTurnController() {
        return swerve.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    protected static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.runOnce(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    private static final SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Uses m/s and rad/s
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                () ->
                        fieldCentricDrive
                                .withVelocityX(fwdPositive.getAsDouble())
                                .withVelocityY(leftPositive.getAsDouble())
                                .withRotationalRate(ccwPositive.getAsDouble()));
    }

    private static final SwerveRequest.RobotCentric robotCentric =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                () ->
                        robotCentric
                                .withVelocityX(fwdPositive.getAsDouble())
                                .withVelocityY(leftPositive.getAsDouble())
                                .withRotationalRate(ccwPositive.getAsDouble()));
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(
                        drive(
                                velocityX,
                                velocityY,
                                () -> swerve.calculateRotationController(targetRadians)))
                .withName("Swerve.aimDrive");
    }

    // TODO: Potential new heading lock, that only fixes minor heading errors, and leaves large ones
    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engague if you are driving x or y.
     */
    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getRotation().getRadians()),
                        drive(
                                velocityX,
                                velocityY,
                                () -> {
                                    if (velocityX.getAsDouble() == 0
                                            && velocityY.getAsDouble() == 0) {
                                        return 0;
                                    } else {
                                        return swerve.calculateRotationController(
                                                () -> config.getTargetHeading());
                                    }
                                }))
                .withName("Swerve.HeadingLock");
    }

    /**
     * *******************************************************************************************
     * Reorient Commands
     */
    protected static Command reorientForward() {
        return swerve.reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    protected static Command reorientLeft() {
        return swerve.reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    protected static Command reorientBack() {
        return swerve.reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    protected static Command reorientRight() {
        return swerve.reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    protected static Command cardinalReorient() {
        return swerve.cardinalReorient().withName("Swerve.cardinalReorient");
    }
}

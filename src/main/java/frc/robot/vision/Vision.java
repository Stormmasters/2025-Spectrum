package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.robot.vision.VisionSystem.Pose2dSupplier;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Trio;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import java.text.DecimalFormat;
import java.util.ArrayList;
import lombok.Getter;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

// TODO: Auto log output may need to be replaced

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision} {@code allLimelights} &
     * {@code poseLimelights}
     */
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String LEFT_LL = "limelight-left";
        public static final LimelightConfig LEFT_Config =
                new LimelightConfig(LEFT_LL)
                        .withTranslation(0, 0, 0.5)
                        .withRotation(0, Math.toRadians(-15), 0);

        public static final String Right_LL = "limelight-right";
        public static final LimelightConfig Right_Config = 
                new LimelightConfig(Right_LL)
                        .withTranslation(0, 0, 0.5)
                        .withRotation(0, Math.toRadians(15), 0);
        // TODO: Limelight config needs to be updated to actual position on robot

        /* Pipeline configs */
        public static final int leftTagPipeline = 0;

        /* Pose Estimation Constants (2024) */

        // Increase these numbers to trust global measurements from vision less. (uses a matrix)
        // (2024)
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs (2024)*/
        // TODO: alignToTag vision &  using pose to align itself

        /*Pose Estimation Constants (2024) */
        public static final double VISION_REJECT_Distance = 1.8;
    } // Excludes Detect LL

    /* Limelights */
    public final Limelight leftLL =
            new Limelight(
                    VisionConfig.LEFT_LL, VisionConfig.leftTagPipeline, VisionConfig.LEFT_Config);
    public final LimelightLogger leftLLogger = new LimelightLogger("left", leftLL);

    public final Limelight[] allLimelights = {leftLL};

    private final DecimalFormat df = new DecimalFormat();

    // @AutoLogOutput(key = "Vision/a_Integrating")
    

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    @Getter boolean isAiming = false;

    @Getter private Pose2dSupplier pose;

    public Vision(Pose2dSupplier pose) {
        setName("vision");
        this.pose = pose;

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
        }

        // removed detectLL(detect Limelight) for now

    }

    @Override
    public void periodic() {}

    // TODO:addFilteredVisionInput method

    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }


    public Limelight getBestLimelight() {
        Limelight bestLimelight = leftLL;
        double bestScore = 0;
        for (Limelight limelight : allLimelights) {
            double score = 0;
            // prefer LL with most tags, when equal tag count, prefer LL closer to tags
            score += limelight.getTagCountInView();
            score += limelight.getTargetSize();

            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }


    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */

    // TODO: Need adding Telemetry prints to each check
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            // replace botpose with this.pose
            Pose2d botpose = botpose3D.toPose2d();
            this.pose = Robot.swerve.getRobotPose();
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when has bad pose
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) { // pose out of field
                Telemetry.print("Pose out of field: " + reject);
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) { // when in air
                Telemetry.print("Pose in air: " + reject);
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when tilted

                Telemetry.print("Pose tilted: " + reject);
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = 0.001;
            VisionConfig.VISION_STD_DEV_Y = 0.001;
            VisionConfig.VISION_STD_DEV_THETA = 0.001;

            // RobotTelemetry would've posted the old the old x, y, and theta values
            Telemetry.print("Vision X: " + df.format(this.pose.getX()) + " Y: " + df.format(this.pose.getY()) + " Theta: " + df.format(this.pose.getRotation().getDegrees()));

            Robot.swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.swerve.addVisionMeasurement(integratedPose, poseTimestamp);
            this.pose =
                    Robot.swerve
                            .getRobotPose(); 
            // get updated pose of x, y, and theta values and then
            Telemetry.print("Vision X: " + df.format(this.pose.getX()) + " Y: " + df.format(this.pose.getY()) + " Theta: " + df.format(this.pose.getRotation().getDegrees()));
            // print "success"
            return true;
        }
        return false; // target not in view
    }

    // TODO:Auton Reset pose to Vision

    // TODO: getBestLimeLight() based on score

    // TODO: targetInView using color pipeline

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (Limelight limelight : allLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    /** Set all Limelights to blink */
    public Command blinkLimelights() {
        return startEnd(
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.blinkLEDs();
                            }
                        },
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.setLEDMode(false);
                            }
                        })
                .withName("Vision.blinkLimelights");
    }

    /** Only blinks left limelight */
    public Command solidLimelight() {
        return startEnd(
                        () -> {
                            leftLL.setLEDMode(true);
                        },
                        () -> {
                            leftLL.setLEDMode(false);
                        })
                .withName("Vision.blinkLimelights");
    }

    // all limelights using the same pipeline

    /** Logging */
    public static class LimelightLogger{
        private final Limelight limelight;
        private String name;

        public LimelightLogger(String name, Limelight limelight) {
            this.limelight = limelight;
            this.name = name;
        }

       
        public boolean getCameraConnection() {
            Telemetry.print(
                    "Vision " + name + " ConnectionStatus: " + limelight.isCameraConnected());
            return limelight.isCameraConnected();
        }

        
        public boolean getIntegratingStatus() { //Vision/Integrating 
            Telemetry.print("Vision " + name + " IntegratingStatus: " + getIntegratingStatus());
            return ;
        }

        public String getLogStatus() { 
            Telemetry.print("Vision " + name + " LogStatus: " + limelight.getLogStatus());
            return limelight.getLogStatus();
        }

        public String getTagStatus() {
            Telemetry.print("Vision " + name + " TagStatus: " + limelight.getTagStatus());
            return limelight.getLogStatus();
        }

        public Pose2d getPose() {
            Telemetry.print("Vision " + name + " Pose: " + limelight.getRawPose3d().toPose2d());
            return limelight.getRawPose3d().toPose2d();
        }

        public Pose2d getMegaPose() {
            Telemetry.print("Vision " + name + " MegaPose: " + limelight.getMegaPose2d());
            return limelight.getMegaPose2d();
        }

        public double getPoseX() {
            Telemetry.print("Vision " + name + " PoseX: " + getPose().getX());
            return getPose().getX();
        }

        public double getPoseY() {
            Telemetry.print("Vision " + name + " PoseY: " + getPose().getY());
            return getPose().getY();
        }

        public double getTagCount() {
            Telemetry.print("Vision " + name + " TagCount: " + limelight.getTagCountInView());
            return limelight.getTagCountInView();
        }

        public double getTargetSize() {
            Telemetry.print("Vision " + name + " TargetSize: " + limelight.getTargetSize());
            return limelight.getTargetSize();
        }
    }

    public static class CommandConfig {
        public double kp;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public CommandConfig alignCommand;
        public double verticalSetpoint; // numbers get small as the cone gets closer
        public double verticalMaxView;

        public void configKp(double kp) {
            this.kp = kp;
        }

        public void configTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        public void configMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void configError(double error) {
            this.error = error;
        }

        public void configPipelineIndex(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public void configLimelight(Limelight limelight) {
            this.limelight = limelight;
        }

        public void configVerticalSetpoint(double verticalSetpoint) {
            this.verticalSetpoint = verticalSetpoint;
        }

        public void configVerticalMaxView(double verticalMaxView) {
            this.verticalMaxView = verticalMaxView;
        }

        public void configAlignCommand(CommandConfig alignCommand) {
            this.alignCommand = alignCommand;
        }

        public CommandConfig() {}
    }
}

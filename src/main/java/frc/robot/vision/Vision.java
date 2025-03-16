package frc.robot.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.Trio;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import lombok.Getter;
import lombok.Setter;

public class Vision extends SubsystemBase implements NTSendable {

    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String FRONT_LL = "limelight-front";
        public static final LimelightConfig FRONT_CONFIG =
                new LimelightConfig(FRONT_LL)
                        .withTranslation(0.215, 0, 0.188)
                        .withRotation(0, Math.toRadians(28), 0);

        public static final String BACK_LL = "limelight-back";
        public static final LimelightConfig BACK_CONFIG =
                new LimelightConfig(BACK_LL)
                        .withTranslation(-0.215, 0.0, 0.188)
                        .withRotation(0, Math.toRadians(28), Math.toRadians(180));

        /* Pipeline configs */
        public static final int frontTagPipeline = 0;
        public static final int backTagPipeline = 0;

        /* Pose Estimation Constants */

        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 0.5;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);
    }

    /** Limelights */
    @Getter
    public final Limelight frontLL =
            new Limelight(
                    VisionConfig.FRONT_LL,
                    VisionConfig.frontTagPipeline,
                    VisionConfig.FRONT_CONFIG);

    public final VisionLogger frontLLLogger = new VisionLogger("front", frontLL);

    public final Limelight backLL =
            new Limelight(
                    VisionConfig.BACK_LL, VisionConfig.backTagPipeline, VisionConfig.BACK_CONFIG);

    public final VisionLogger backLLLogger = new VisionLogger("back", backLL);

    public final Limelight[] allLimelights = {frontLL, backLL};

    public final VisionLogger[] allLimelightLoggers = {frontLLLogger, backLLLogger};

    private final DecimalFormat df = new DecimalFormat();

    @Getter @Setter private boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    @Getter private boolean isAiming = false;

    int[] blueTags = {17, 18, 19, 20, 21, 22};
    int[] redTags = {6, 7, 8, 9, 10, 11};

    public Vision() {
        setName("vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
            limelight.setIMUmode(3);
        }

        SendableRegistry.add(this, getName());
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();

        for (Limelight limelight : allLimelights) {
            limelight.setRobotOrientation(yaw);
        }

        if (Util.disabled.getAsBoolean()) {
            for (Limelight limelight : allLimelights) {
                limelight.setIMUmode(1);
            }
            try {
                addMegaTag1_VisionInput(backLL, true);
            } catch (Exception e) {
                Telemetry.print("REAR MT1: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInput(frontLL, true);
            } catch (Exception e) {
                Telemetry.print("FRONT MT1: Vision pose not present but tried to access it");
            }
        }

        if (Util.teleop.getAsBoolean()) {
            for (Limelight limelight : allLimelights) {
                limelight.setIMUmode(3);
            }
            try {
                addMegaTag2_VisionInput(backLL);
            } catch (Exception e) {
                Telemetry.print("REAR MT2: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag2_VisionInput(frontLL);
            } catch (Exception e) {
                Telemetry.print("FRONT MT2: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInput(backLL, false);
            } catch (Exception e) {
                Telemetry.print("REAR MT1: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInput(frontLL, false);
            } catch (Exception e) {
                Telemetry.print("FRONT MT1: Vision pose not present but tried to access it");
            }
        }
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("VisionTargetValues");
        builder.addDoubleProperty("FrontTX", frontLL::getTagTx, null);
        builder.addDoubleProperty("FrontTY", frontLL::getTagTA, null);
        builder.addDoubleProperty("FrontRotation", frontLL::getTagRotationDegrees, null);
    }

    private void addMegaTag1_VisionInput(Limelight ll, boolean integrateXY) {
        double xyStds = 1000;
        double degStds = 1000;

        boolean mt1_reject = false;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getMegaTag1PoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d megaTag1_3d = ll.getMegaTag1_Pose3d();
            Pose2d megaTag1_2d = megaTag1_3d.toPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();

            // distance from current pose to vision estimated MT2 pose
            double mt1_poseDifference =
                    Robot.getSwerve()
                            .getRobotPose()
                            .getTranslation()
                            .getDistance(megaTag1_2d.getTranslation());

            /* rejections */
            // reject mt1 pose if individual tag ambiguity is too high
            ll.setTagStatus("");
            for (RawFiducial tag : tags) {
                // search for highest ambiguity tag for later checks
                if (highestAmbiguity == 2) {
                    highestAmbiguity = tag.ambiguity;
                } else if (tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                // log ambiguities
                // ll.setTagStatus("Tag " + tag.id + ": " + tag.ambiguity);
                // ambiguity rejection check
                if (tag.ambiguity > 0.9) {
                    // ll.sendInvalidStatus("ambiguity rejection");
                    mt1_reject = true;
                }
            }

            if (Field.poseOutOfField(megaTag1_3d)) {
                // reject if pose is out of the field
                mt1_reject = true;
                ll.sendInvalidStatus("bound rejection");
            }

            if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 1.6) {
                // reject if we are rotating more than 0.5 rad/s
                ll.sendInvalidStatus("rotation rejection");
                mt1_reject = true;
            }

            if (Math.abs(megaTag1_3d.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                ll.sendInvalidStatus("height rejection");
                mt1_reject = true;
            }

            if (Math.abs(megaTag1_3d.getRotation().getX()) > 5
                    || Math.abs(megaTag1_3d.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("roll/pitch rejection");
                mt1_reject = true;
            }

            if (targetSize <= 0.025) {
                ll.sendInvalidStatus("size rejection");
                mt1_reject = true;
            }

            /* integrations */
            // if almost stationary and extremely close to tag
            if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 0.1) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
                degStds = 8;
                if (targetSize > 2) {
                    ll.sendValidStatus("Strong Multi integration");
                    xyStds = 0.1;
                    degStds = 0.1;
                }
            } else if (targetSize > 0.8
                    && (mt1_poseDifference < 0.5 || DriverStation.isDisabled())) {
                // Integrate if the target is very big and we are close to pose or disabled
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 999999;
            } else if (targetSize > 0.1
                    && (mt1_poseDifference < 0.25 || DriverStation.isDisabled())) {
                // Integrate if we are very close to pose or disabled and target is large enough
                ll.sendValidStatus("Proximity integration");
                xyStds = 1.0;
                degStds = 999999;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = 1.5;
                degStds = 999999;
            } else {
                // Shouldn't integrate
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = 15;
            }

            if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
                degStds = 50;
            }

            if (mt1_reject) {
                xyStds = 999999;
                degStds = 999999;
            }

            if (!integrateXY) {
                xyStds = 999999;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(
                            VecBuilder.fill(
                                    VisionConfig.VISION_STD_DEV_X,
                                    VisionConfig.VISION_STD_DEV_Y,
                                    VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose =
                    new Pose2d(megaTag1_2d.getTranslation(), megaTag1_2d.getRotation());
            Robot.getSwerve()
                    .addVisionMeasurement(integratedPose, Utils.fpgaToCurrentTime(timeStamp));
        } else {
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    private void addMegaTag2_VisionInput(Limelight ll) {
        double xyStds = 1000;
        double degStds = 99999;

        boolean mt2_reject = false;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getMegaTag2PoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose2d megaTag2_2d = ll.getMegaTag2_Pose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();

            // distance from current pose to vision estimated MT2 pose
            double mt2_poseDifference =
                    Robot.getSwerve()
                            .getRobotPose()
                            .getTranslation()
                            .getDistance(megaTag2_2d.getTranslation());

            /* rejections */
            if (Field.poseOutOfField(megaTag2_2d)) {
                // reject if pose is out of the field
                mt2_reject = true;
                ll.sendInvalidStatus("bound rejection");
            }

            if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 1.6) {
                // reject if we are rotating more than 0.5 rad/s
                ll.sendInvalidStatus("rotation rejection");
                mt2_reject = true;
            }

            if (targetSize <= 0.025) {
                ll.sendInvalidStatus("size rejection");
                mt2_reject = true;
            }

            /* integrations */
            // if almost stationary and extremely close to tag
            if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 0.1) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
                if (targetSize > 2) {
                    ll.sendValidStatus("Strong Multi integration");
                    xyStds = 0.1;
                }
            } else if (targetSize > 0.8
                    && (mt2_poseDifference < 0.5 || DriverStation.isDisabled())) {
                // Integrate if the target is very big and we are close to pose or disabled
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
            } else if (targetSize > 0.1
                    && (mt2_poseDifference < 0.25 || DriverStation.isDisabled())) {
                // Integrate if we are very close to pose or disabled and target is large enough
                ll.sendValidStatus("Proximity integration");
                xyStds = 0.0;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = 0.5;
            } else {
                // Shouldn't integrate
                return;
            }

            if (mt2_reject) {
                xyStds = 999999;
                return;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(
                            VecBuilder.fill(
                                    VisionConfig.VISION_STD_DEV_X,
                                    VisionConfig.VISION_STD_DEV_Y,
                                    VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose =
                    new Pose2d(megaTag2_2d.getTranslation(), megaTag2_2d.getRotation());
            Robot.getSwerve()
                    .addVisionMeasurement(
                            integratedPose,
                            Utils.fpgaToCurrentTime(timeStamp),
                            VecBuilder.fill(
                                    VisionConfig.VISION_STD_DEV_X,
                                    VisionConfig.VISION_STD_DEV_Y,
                                    VisionConfig.VISION_STD_DEV_THETA));
        } else {
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    /**
     * Choose the limelight with the best view of multiple tags
     *
     * @return
     */
    public Limelight getBestLimelight() {
        Limelight bestLimelight = frontLL;
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

    /** reset pose to the best limelight's vision pose */
    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(),
                ll.getMegaTag1_Pose3d(),
                ll.getMegaTag2_Pose2d(),
                ll.getMegaTag1PoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {

        boolean reject = false;
        if (targetInView) {
            // replace botpose with this.pose
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d pose = Robot.getSwerve().getRobotPose();

            // Check if the vision pose is bad and don't trust it
            if (Field.poseOutOfField(botpose3D)) { // pose out of field
                Telemetry.log("Pose out of field", reject);
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) { // when in air
                Telemetry.log("Pose in air", reject);
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when tilted

                Telemetry.log("Pose tilted", reject);
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs
            double VISION_STD_DEV_X = 0.00001;
            double VISION_STD_DEV_Y = 0.00001;
            double VISION_STD_DEV_THETA = 0.00001;

            // Posts Current X,Y, and Angle (Theta) values
            double[] visionPose = {
                botpose.getX(), botpose.getY(), botpose.getRotation().getDegrees()
            };
            Telemetry.log("Current Vision Pose: ", visionPose);

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(
                            VecBuilder.fill(
                                    VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.getSwerve().addVisionMeasurement(integratedPose, poseTimestamp);
            pose = Robot.getSwerve().getRobotPose();
            // Gets updated pose of x, y, and theta values
            visionPose = new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
            Telemetry.log("Vision Pose Reset To: ", visionPose);

            // print "success"
            return true;
        }
        return false; // target not in view
    }

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

    // ------------------------------------------------------------------------------
    // Calculation Functions
    // ------------------------------------------------------------------------------

    /**
     * Get the angle the robot should turn to based on the id the limelight is seeing.
     *
     * @return
     */
    public double getReefTagAngle() {
        double[][] reefFrontAngles = {
            {17, 60}, {18, 0}, {19, -60}, {20, -120}, {21, 180}, {22, 120},
            {6, 120}, {7, 180}, {8, -120}, {9, -60}, {10, 0}, {11, 60}
        };

        int closetFrontTag = (int) frontLL.getClosestTagID();
        int closetRearTag = (int) backLL.getClosestTagID();
        int closetTag = closetFrontTag;
        boolean rearTag = false;

        if (closetTag == -1) {
            closetTag = closetRearTag;
            rearTag = true;
        }

        if (closetTag == -1) {
            // Return current angle if no tag seen before going through the array
            return Robot.getSwerve().getRobotPose().getRotation().getRadians();
        }

        for (int i = 0; i < reefFrontAngles.length; i++) {
            if (closetTag == reefFrontAngles[i][0]) {
                if (rearTag) {
                    return Math.toRadians(reefFrontAngles[i][1] + 180);
                }
                return Math.toRadians(reefFrontAngles[i][1]);
            }
        }

        // Return current angle if no tag is found
        return Robot.getSwerve().getRobotPose().getRotation().getRadians();
    }

    public boolean tagsInView() {

        DriverStation.Alliance alliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);

        if (alliance == DriverStation.Alliance.Blue) {
            double closestTagIDFront = frontLL.getClosestTagID();
            double closestTagIDBack = backLL.getClosestTagID();

            boolean isFrontTagInBlue =
                    Arrays.stream(blueTags).anyMatch(tag -> tag == closestTagIDFront);
            boolean isBackTagInBlue =
                    Arrays.stream(blueTags).anyMatch(tag -> tag == closestTagIDBack);

            if (isFrontTagInBlue || isBackTagInBlue) {
                return true;
            } else {
                return false;
            }
        } else if (alliance == DriverStation.Alliance.Red) {
            double closestTagIDFront = frontLL.getClosestTagID();
            double closestTagIDBack = backLL.getClosestTagID();

            boolean isFrontTagInRed =
                    Arrays.stream(redTags).anyMatch(tag -> tag == closestTagIDFront);
            boolean isBackTagInRed =
                    Arrays.stream(redTags).anyMatch(tag -> tag == closestTagIDBack);

            if (isFrontTagInRed || isBackTagInRed) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public double getTagTA() {
        if (frontLL.targetInView()) {
            return frontLL.getTagTA();
        } else if (backLL.targetInView()) {
            return backLL.getTagTA();
        } else {
            return 0;
        }
    }

    public double getTagTX() {
        if (frontLL.targetInView()) {
            return frontLL.getTagTx();
        } else if (backLL.targetInView()) {
            return backLL.getTagTx();
        } else {
            return 0;
        }
    }

    // ------------------------------------------------------------------------------
    // VisionStates Commands
    // ------------------------------------------------------------------------------

    /** Set all Limelights to blink */
    public Command blinkLimelights() {
        Telemetry.print("Vision.blinkLimelights", PrintPriority.HIGH);
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
                            frontLL.setLEDMode(true);
                        },
                        () -> {
                            frontLL.setLEDMode(false);
                        })
                .withName("Vision.solidLimelight");
    }
}

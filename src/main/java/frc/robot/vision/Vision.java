package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.spectrumLib.util.Trio;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import lombok.Getter;
import frc.reefscape.Field;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import frc.spectrumLib.Telemetry;



// TODO: Auto log output may need to be replaced 

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision} {@code allLimelights} &
     * {@code poseLimelights}
     */
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String LEFT_LL = "limelight-left";
        public static final LimelightConfig LEFT_Config = new LimelightConfig(LEFT_LL)
            .withTranslation(0, 0, 0.5)
            .withRotation(0, Math.toRadians(-15), 0); 
            //TODO: Limelight config needs to be updated to actual position on robot
        
        public static final String RIGHT_LL = "limelight-right";
        public static final LimelightConfig RIGHT_Config = new LimelightConfig(RIGHT_LL)
            .withTranslation(0, 0, 0.5)
            .withRotation(0, Math.toRadians(15), 0);
        /* Pipeline configs */
        public static final int leftTagPipeline = 0; 

        /* Pose Estimation Constants (2024) */

        // Increase these numbers to trust global measurements from vision less. (uses a matrix) (2024)
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
            VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs (2024)*/ //TODO: alignToTag vision &  using pose to align itself
    
        /*Pose Estimation Constants (2024) */
        public static final double VISION_REJECT_Distance = 1.8;
    }

    /* Limelights */
    public final Limelight leftLL = new Limelight(VisionConfig.LEFT_LL, VisionConfig.leftTagPipeline, VisionConfig.LEFT_Config);  
    public final LimelightLogger leftLLogger = new LimelightLogger("left", leftLL);
   
    public final Limelight[] allLimelights = {leftLL}; 

    private final DecimalFormat df = new DecimalFormat();

    // @AutoLogOutput(key = "Vision/a_Integrating")
    public static boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    @Getter boolean isAiming = false;

    @Getter private Pose2d pose;
    public Vision(Pose2d pose) {
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
    public void periodic() {
        
       
    }

    //TODO:addFilteredVisionInput method

  
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

    
    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */

     //TODO: Need adding Telemetry prints to each check
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            //replace botpose with this.pose
            Pose2d botpose = botpose3D.toPose2d();
            this.pose = Robot.swerve.getRobotPose();
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) { //when has bad pose
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) { //pose out of field
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) { //when in air
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) { //when tilted
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

            //RobotTelemetry would've posted the old the old x, y, and theta values
            Robot.swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.swerve.addVisionMeasurement(integratedPose, poseTimestamp);
            this.pose = Robot.swerve.getRobotPose(); // get updated pose of x, y, and theta values and then print "success"
            return true;
        }
        return false; // target not in view
    }

    //TODO:Auton Reset pose to Vision
    
    ////TODO: getBestLimeLight() based on score

    //TODO: targetInView using color pipeline

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

    /**Set all Limelights to blink */
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

    /** Only blinks left limelight*/
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
    
    
    
    
    //all limelights using the same pipeline
    

   

    /** Logging */
    public static class LimelightLogger {
        private final Limelight limelight;
        private String name;

        public LimelightLogger(String name, Limelight limelight) {
            this.limelight = limelight;
            this.name = name;
        }

        
        public boolean getCameraConnection() {
            return limelight.isCameraConnected();
        }

        
        public boolean getIntegratingStatus() {
            Telemetry.print("Vision " + name + " LogStatus " + VisionConfig.LEFT_Config.isIntegrating());
            return VisionConfig.LEFT_Config.isIntegrating();
        }

       
        public String getLogStatus() {
            Telemetry.print("Vision " + name + " LogStatus " + limelight.getLogStatus());
            return limelight.getLogStatus();
        }

        
        public String getTagStatus() {
            Telemetry.print("Vision " + name + " TagStatus " + limelight.getTagStatus());
            return limelight.getTagStatus();
        }

        
        public Pose2d getPose() {
            Telemetry.print(name + " Pose " + limelight.getRawPose3d().toPose2d());
            return limelight.getRawPose3d().toPose2d();
        }

    
        public Pose2d getMegaPose() {
            Telemetry.print(name + " MegaPose " + limelight.getMegaPose2d());
            return limelight.getMegaPose2d();
        }

   
        public double getPoseX() {
            Telemetry.print(name + " PoseX " + limelight.getRawPose3d().toPose2d().getX());
            return getPose().getX();
        }

     
        public double getPoseY() {
            Telemetry.print(name + " PoseY " + limelight.getRawPose3d().toPose2d().getY());
            return getPose().getY();
        }

        public double getTagCount() {
            Telemetry.print(name + " TagCount " + limelight.getTagCountInView());
            return limelight.getTagCountInView();
        }

       
        public double getTargetSize() {
            Telemetry.print(name + " TargetSize " + limelight.getTargetSize());
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

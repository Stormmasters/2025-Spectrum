package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.spectrumLib.util.Trio;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.LimelightHelpers;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision} {@code allLimelights} &
     * {@code poseLimelights}
     */
    public static final class VisionConfig {
        /* Limelight Configuration */

        /* Pipeline configs */
       
        /* Pose Estimation Constants */

        // Increase these numbers to trust global measurements from vision less. (uses a matrix)

        /* Vision Command Configs */
       
    }

    /* Limelights */
    

    private final DecimalFormat df = new DecimalFormat();

    // @AutoLogOutput(key = "Vision/a_Integrating")
    public static boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    private boolean isAiming = false;

    public Vision() {
        setName("Vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
        }

        detectLL.setLEDMode(true);
    }

    @Override
    public void periodic() {
        
       
    }

    //TODO:addFilteredVisionInput

    //TODO: Reset Pose to Vision Method
    //TODO:Auton Reset pose to Vision
    
    ////TODO: getBestLimeLight() based on score

    //TODO: targetInView using color pipeline

    //TODO: hasAccuratePose (PoseEstimation)

    //TODO: method to blink limelights to turn thier LEDS on and off

    //all limelights using the same pipeline

    //Getter for is aiming variable
    

   

    /** Logging */

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

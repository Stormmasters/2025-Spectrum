package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;


public class VisionStates {
    //TODO: needs aligntovision and drivetovisiontarget methods
    
    private static Vision vision = Robot.getVision();

    public static void setupDefaultCommand() {
        vision.setDefaultCommand(blinkLimelights());
    }

    public static void setStates() {
        
    }

    public static Command blinkLimelights() {
        return vision.blinkLimelights().withName("VisionStates.blinkLimelights");
    }

    public static Command solidLimelight() {
        return vision.solidLimelight().withName("VisionCommands.solidLimelight");
    }
    /** Set robot pose to vision pose only if LL has good tag reading */
    public static Command resetPoseToVision() {
        return vision.runOnce(vision::resetPoseToVision)
                .withName("VisionCommands.resetPoseToVision");
    }

    /** Set robot pose to vision pose looking at 5 of the last available poses in auto */
    public static Command autonResetPoseToVision() {
        return vision.runOnce(vision::autonResetPoseToVision)
                .withName("VisionCommands.autonResetPoseToVision");
    }

    /** Set robot pose to vision pose regardless of validity. Does not reset rotation. */
    public static Command forcePoseToVision() {
        return vision.run(vision::forcePoseToVision).withName("VisionCommands.forcePoseToVision");
    }
}
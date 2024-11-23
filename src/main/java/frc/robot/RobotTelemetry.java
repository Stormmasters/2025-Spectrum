package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.spectrumLib.Telemetry;
import java.text.DecimalFormat;

public class RobotTelemetry extends Telemetry {

    /* Truncating double logs */
    private static final DecimalFormat df = new DecimalFormat();

    /* What to publish over networktables for telemetry */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Keep a reference of the last pose to calculate the speeds */
    Pose2d m_lastPose = new Pose2d();
    double lastTime = Utils.getCurrentTimeSeconds();

    public RobotTelemetry() {
        super();
        enablePrints();
        df.setMaximumFractionDigits(2);
    }

    public static String truncatedDouble(double number) {
        return df.format(number);
    }
}

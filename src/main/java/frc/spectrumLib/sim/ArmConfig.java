package frc.spectrumLib.sim;

import lombok.Getter;
import lombok.Setter;

public class ArmConfig {

    @Getter @Setter private int numMotors = 1;
    @Getter @Setter private double initialX = 0.7;
    @Getter @Setter private double initialY = 0.3;
    @Getter @Setter private double pivotX = 0.7;
    @Getter @Setter private double pivotY = 0.3;
    @Getter @Setter private double ratio = 50;
    @Getter @Setter private double length = 0.5;
    @Getter @Setter private double simMOI = 1.2;
    @Getter @Setter private double simCGLength = 0.2;
    @Getter @Setter private double minAngle = Math.toRadians(-60);
    @Getter @Setter private double maxAngle = Math.toRadians(90);
    @Getter @Setter private double startingAngle = Math.toRadians(90);
    @Getter @Setter private boolean simulateGravity = true;
    @Getter @Setter private double initialAttachedX;
    @Getter @Setter private double initialAttachedY;
    @Getter private boolean mounted = false;
    @Getter private Mount mount;
    @Getter private double mountX;
    @Getter private double mountY;

    public ArmConfig(
            double initialX,
            double initialY,
            double ratio,
            double length,
            double minAngleDegrees,
            double maxAngleDegrees,
            double startingAngleDegrees) {
        this.ratio = ratio;
        this.length = length;
        this.minAngle = Math.toRadians(minAngleDegrees);
        this.maxAngle = Math.toRadians(maxAngleDegrees);
        this.startingAngle = Math.toRadians(startingAngleDegrees);
        this.initialX = initialX;
        this.initialY = initialY;
        this.pivotX = initialX;
        this.pivotY = initialY;
    }

    public ArmConfig setMount(LinearSim sim) {
        mounted = true;
        mount = sim;
        mountX = sim.getConfig().getInitialX();
        mountY = sim.getConfig().getInitialY();
        return this;
    }

    public ArmConfig setMount(ArmSim sim) {
        mounted = true;
        mount = sim;
        mountX = sim.getConfig().getInitialX();
        mountY = sim.getConfig().getInitialY();
        return this;
    }
}

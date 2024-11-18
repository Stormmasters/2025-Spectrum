package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RollerSim {

    private MechanismRoot2d rollerAxle;
    private MechanismLigament2d[] rollerBackground; // TODO: should figure out why we don't use this
    private MechanismLigament2d rollerViz;

    private FlywheelSim rollerSim;
    private TalonFXSimState rollerMotorSim;
    private RollerConfig config;
    private Circle roller;

    public RollerSim(
            RollerConfig config, Mechanism2d mech, TalonFXSimState rollerMotorSim, String name) {
        this.config = config;
        this.rollerMotorSim = rollerMotorSim;
        DCMotor kraken = DCMotor.getKrakenX60Foc(1);
        LinearSystem<N1, N1, N1> flyWheelSystem =
                LinearSystemId.createFlywheelSystem(
                        kraken, config.getSimMOI(), config.getGearRatio());
        rollerSim = new FlywheelSim(flyWheelSystem, kraken);

        rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);

        rollerViz =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(config.getRollerDiameterInches()) / 2.0,
                                0.0,
                                5.0,
                                new Color8Bit(Color.kWhite)));

        rollerBackground = new MechanismLigament2d[config.getBackgroundLines()];

        roller =
                new Circle(
                        config.getBackgroundLines(),
                        config.getRollerDiameterInches(),
                        name,
                        rollerAxle);
    }

    public void simulationPeriodic() { // double x, double y) {
        // ------ Update sim based on motor output
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(TimedRobot.kDefaultPeriod);

        // ------ Update motor based on sim
        // Make sure to convert radians at the mechanism to rotations at the motor
        // Subtracting out the starting angle is necessary so the simulation can't "cheat" and use
        // the
        // sim as an absolute encoder.
        double rotationsPerSecond = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        rollerMotorSim.setRotorVelocity(rotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotationsPerSecond * TimedRobot.kDefaultPeriod);

        // Update the axle as the robot moves
        if (config.isMounted()) {
            rollerAxle.setPosition(getUpdatedX(), getUpdatedY());
        } else {
            rollerAxle.setPosition(config.getInitialX(), config.getInitialY());
        }

        // Scale down the angular velocity so we can actually see what is happening
        double rpm = rollerSim.getAngularVelocityRPM() / 2;
        rollerViz.setAngle(
                rollerViz.getAngle() + Math.toDegrees(rpm) * TimedRobot.kDefaultPeriod * 0.1);

        if (rollerSim.getAngularVelocityRadPerSec() < -1) {
            roller.setHalfBackground(config.getRevColor(), config.getOffColor());
        } else if (rollerSim.getAngularVelocityRadPerSec() > 1) {
            roller.setBackgroundColor(config.getFwdColor());
        } else {
            roller.setBackgroundColor(config.getOffColor());
        }
    }

    public double getUpdatedX() {
        switch (config.getMount().getMountType()) {
            case LINEAR:
                return config.getInitialX() + config.getMount().getDisplacementX();
            case ARM:
                return Math.sqrt(
                                        Math.pow(config.getInitialX() - config.getMountX(), 2)
                                                + Math.pow(
                                                        config.getInitialY() - config.getMountY(),
                                                        2))
                                * Math.cos(config.getMount().getAngle())
                        + config.getMount().getDisplacementX()
                        + config.getMountX();
            default:
                return 0;
        }
    }

    public double getUpdatedY() {
        switch (config.getMount().getMountType()) {
            case LINEAR:
                return config.getInitialY() + config.getMount().getDisplacementY();
            case ARM:
                return Math.sqrt(
                                        Math.pow(config.getInitialX() - config.getMountX(), 2)
                                                + Math.pow(
                                                        config.getInitialY() - config.getMountY(),
                                                        2))
                                * Math.sin(config.getMount().getAngle())
                        + config.getMount().getDisplacementY()
                        + config.getMountY();
            default:
                return 0;
        }
    }

    // public void setBackgroundColor(Color8Bit color8Bit) {
    //     for (int i = 0; i < config.getBackgroundLines(); i++) {
    //         rollerBackground[i].setColor(color8Bit);
    //     }
    // }

    // public void setHalfBackground(Color8Bit color8Bit) {
    //     for (int i = 0; i < config.getBackgroundLines(); i++) {
    //         if (i % 2 == 0) {
    //             rollerBackground[i].setColor(color8Bit);
    //         } else {
    //             rollerBackground[i].setColor(config.getOffColor());
    //         }
    //     }
    // }
}

package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

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

        rollerBackground = new MechanismLigament2d[config.getBackgroundLines()];

        roller =
                new Circle(
                        config.getBackgroundLines(),
                        config.getRollerDiameterInches(),
                        name,
                        rollerAxle,
                        mech);
    }

    public void simulationPeriodic(double x, double y) {
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
        rollerAxle.setPosition(x, y);

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

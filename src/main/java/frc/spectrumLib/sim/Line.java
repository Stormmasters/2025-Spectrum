package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.spectrumLib.sim.Mount.MountType;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;


public class Line {
    private SingleJointedArmSim armSim;
    @Getter private ArmConfig config;

    private MechanismRoot2d armPivot;
    private MechanismLigament2d armMech2d;
    private TalonFXSimState armMotorSim;
    private String name;
    private Mechanism2d mech;
    private double pivotX;
    private double pivotY;
    private double minAngle;
    private double length;
    private double width;
    private Color8Bit color;

    @Getter private MountType mountType = MountType.ARM;
    public Line(double length, double width, double minAngle, double pivotX, double pivotY, String name, Color8Bit color){
        this.length=length;
        this.minAngle=minAngle;
        this.pivotX=pivotX;
        this.pivotY=pivotY;
        this.name=name;
        this.width=width;
        this.color=color;
        drawLine();
    }
    public void drawLine(){
        armPivot = mech.getRoot(name + " Arm Pivot", pivotX, pivotY);
        armMech2d =
               armPivot.append(
                        new MechanismLigament2d(
                                name + " Arm",
                                length,
                                minAngle,
                                width,
                                color));
    }
    
}

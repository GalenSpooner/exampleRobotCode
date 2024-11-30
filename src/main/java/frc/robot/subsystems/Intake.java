package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    TalonFX intakeMotor;
    TalonFX pivotMotor;
    DigitalInput cone;
    DigitalInput cube;
    intakeState currentState;
    PivotState pivotState;


    public static enum intakeState{
        INTAKING,
        IDLE,
        CONE,
        CUBE,
        OUTTAKING

    }
    public static enum PivotState{
        OUT(Constants.IntakeCosntants.kExtendedDegrees),
        IN(Constants.IntakeCosntants.kStowDegrees);

        public final double position;
        private PivotState(double position) {
            this.position = position;
        }
    }
    public Intake(){
        intakeMotor = new TalonFX(Constants.IntakeCosntants.kIntakeID);
        cone = new DigitalInput(Constants.IntakeCosntants.kBeamBreakID);
        cube = new DigitalInput(Constants.IntakeCosntants.kBeamBreakID);
        currentState = intakeState.IDLE;
        pivotState = PivotState.IN;
        pivotMotor.getConfigurator().apply(new Slot0Configs().withKA(0).withKP(0.1).withKS(.154).withKG(.15));
        pivotMotor.getConfigurator().apply(new MotionMagicConfigs().withMotionMagicAcceleration(50).withMotionMagicCruiseVelocity(100).withMotionMagicExpo_kA(0).withMotionMagicExpo_kV(0.25));
        
    }
    @Override
    public void periodic() {
        this.currentState = (cone.get()) ? intakeState.CONE : (cube.get()) ? intakeState.CUBE : this.currentState;
        switch(currentState){
            case IDLE:
                intakeMotor.set(0);
                break;
            case CONE:
                intakeMotor.set(-.1);
                break;
            case CUBE:
                intakeMotor.set(-.1);
                break;
            default:
                break;
        }
        pivotMotor.setControl(new MotionMagicExpoDutyCycle(pivotState.position));

        }
    
    public Command setState(intakeState newState){
        return run(() -> {
            this.currentState = newState;
        });
    }
    public Command setPivot(PivotState newState){
        return run(() -> {
            this.pivotState = newState;
        });
    }

}

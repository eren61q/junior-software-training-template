import main.java.frc.robot.Constants;

public double targetClimbPosition;

public class ClimbSubsystem{    

SparkMax masterMotor = new SparkMax(Constants.Climb.masterMotor, MotorType.kBrushless);
SparkMax slaveMotor = new SparkMax(Constants.Climb.slaveMotor, MotorType.kBrushless);

private final absolutencoder*** masterEncoder = masterMotor.getEncoder();
private double targetPosition;

    public void openClimb(){
        masterMotor.setVoltage(12);
        slaveMotor.setVoltage(12);
            
}
    public void closeClimb(){
        masterMotor.setVoltage(-12);
        slaveMotor.setVoltage(-12);
            
}

    }
    public enum ClimbState {
        IDLE,
        OPENING,
        CLOSING,
        HOLDING
    }

    public void setState(ClimbState newState) {
        state = newState;
        if (state == ClimbState.HOLDING) {
            targetPosition = encoder.getPosition(); // hold current position
        } else if (state == ClimbState.OPENING) {
            targetPosition += 1.0; // number depends on target position
        } else if (state == ClimbState.CLOSING) {
            targetPosition -= 1.0;
        }
    }



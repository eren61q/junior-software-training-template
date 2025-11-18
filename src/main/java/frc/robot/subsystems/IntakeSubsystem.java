package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.subsystems.ElevatorSubsystem.ControlMode;

@Logged(name = "Intake")
public class IntakeSubsystem extends SubsystemBase {

    SparkMax intakeMotor;
    DigitalInput sensor;

    public SparkMaxConfig SparkMaxConfig() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
        return motorConfig;
    }

    public IntakeSubsystem() {
        sensor = new DigitalInput(Constants.Intake.sensorID);
        intakeMotor = new SparkMax(Constants.Intake.motorID, MotorType.kBrushless);
    }

    public double getVoltage() {
        return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    }

    public void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public boolean hasCoral() {
        return sensor.get();
    }

    public void periodic() {
        SmartDashboard.putBoolean("hasCoral", hasCoral());
    }

    //kick out
    public Command intakeOut(){
      return run(() -> setVoltage(-12)).until(() -> !hasCoral()).finallyDo(interrupted -> stop()); 
    }

    //default intake
    public Command intakeIn(){
      return run(() -> setVoltage(12)).until(() -> hasCoral()).finallyDo(interrupted -> stop());
    }


}
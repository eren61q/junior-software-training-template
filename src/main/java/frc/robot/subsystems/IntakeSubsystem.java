package frc.robot.subsystems;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.subsystems.ElevatorSubsystem.ControlMode;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

@Logged(name = "Intake")
public class IntakeSubsystem extends SubsystemBase {

    SparkMax intakeMotor;
    DigitalInput sensor;

    public SparkMaxConfig SparkMaxConfig() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
        return motorConfig;
    }

    public IntakeSubsystem(){
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
    public void periodic(){
        SmartDashboard.putBoolean("hasCoral", hasCoral());
    }
}
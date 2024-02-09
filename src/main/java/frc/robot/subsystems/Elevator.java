package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Elevator extends PIDSubsystem {
    private TalonFX elevatorMotorLeft = new TalonFX(44); 
    private TalonFX elevatorMotorRight = new TalonFX(45); 
    
    AnalogInput input = new AnalogInput(1);
    AnalogPotentiometer pot = new AnalogPotentiometer(input, 51.6, 1.6);

    CommandXboxController operatorJoystick;

    double elevatorMaxSetPoint = 10;
    double elevatorMinSetPoint = 1.6;

    public Elevator(CommandXboxController operatorJoystick) {
        super(new PIDController(0.1, 0.0, 0.0001));
        this.operatorJoystick = operatorJoystick;
        getController().setTolerance(2.0);
        setSetpoint(pot.get());
        enable();

        var setElevatorMotorLeftToDefault = new TalonFXConfiguration();
        elevatorMotorLeft.getConfigurator().apply(setElevatorMotorLeftToDefault);

        var setElevatorMotorRightToDefault = new TalonFXConfiguration();
        elevatorMotorRight.getConfigurator().apply(setElevatorMotorRightToDefault);
        
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Voltage.PeakForwardVoltage = 8;
        leftMotorConfig.Voltage.PeakReverseVoltage = -8;

        elevatorMotorLeft.setSafetyEnabled(false);
        elevatorMotorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.Voltage.PeakForwardVoltage = 8;
        rightMotorConfig.Voltage.PeakReverseVoltage = -8;
        
        elevatorMotorRight.setSafetyEnabled(false);
        elevatorMotorRight.getConfigurator().apply(rightMotorConfig);
        
    }

    public void setElevatorSetpoint(double setPoint) {
            if (setPoint < elevatorMinSetPoint) {
                setPoint = elevatorMinSetPoint;
            } else if (setPoint > elevatorMaxSetPoint) {
                setPoint = elevatorMaxSetPoint;
            }
            this.setSetpoint(setPoint);
    }

    public void enablePID(boolean override) {
        if (override) {
            enable();
        } else {
            disable();
        }
    }
    
    public void setElevatorSpeed(double speed) {
        elevatorMotorLeft.set(speed);
        elevatorMotorRight.set(speed);
    }

    public void stopMotors() {
        elevatorMotorLeft.stopMotor();
        elevatorMotorRight.stopMotor();
    }

    public double getElevatorLeftTemp() {
        return elevatorMotorLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getElevatorRightTemp() {
        return elevatorMotorRight.getDeviceTemp().getValueAsDouble();
    }

    public void getSetSetPoint() {
        setSetpoint(pot.get());
    }

    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
            SmartDashboard.putNumber("Elevator/Left Motor Temperature", getElevatorLeftTemp());
            SmartDashboard.putNumber("Elevator/Right Motor Temperature", getElevatorRightTemp());
            SmartDashboard.putNumber("Elevator Position", pot.get());
            SmartDashboard.putNumber("Elevator Set Point", getSetpoint());
            SmartDashboard.putBoolean("Elevator at Set Point", getController().atSetpoint());
            
        }
    }

    public void initialize() {
        setSetpoint(pot.get());
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (output > Constants.Elevator.ELEVATOR_MAX_PID_SPEED) {
            output  = Constants.Elevator.ELEVATOR_MAX_PID_SPEED;
        } else if (output < -Constants.Elevator.ELEVATOR_MAX_PID_SPEED) {
            output = -Constants.Elevator.ELEVATOR_MAX_PID_SPEED;
        }
        setElevatorSpeed(output);
        SmartDashboard.putNumber("Elevator Output", output);
    }

    @Override
    protected double getMeasurement() {
       return pot.get();
    }

   
}

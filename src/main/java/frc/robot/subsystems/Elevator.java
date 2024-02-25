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
    private TalonFX elevatorMotor = new TalonFX(44);
    
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

        var setElevatorMotorToDefault = new TalonFXConfiguration();
        elevatorMotor.getConfigurator().apply(setElevatorMotorToDefault);
        
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Voltage.PeakForwardVoltage = 8;
        leftMotorConfig.Voltage.PeakReverseVoltage = -8;

        elevatorMotor.setSafetyEnabled(false);
        elevatorMotor.getConfigurator().apply(leftMotorConfig);

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
        elevatorMotor.set(speed);
    }

    public void stopMotors() {
        elevatorMotor.stopMotor();
    }

    public double getElevatorTemp() {
        return elevatorMotor.getDeviceTemp().getValueAsDouble();
    }

    public void getSetSetPoint() {
        setSetpoint(pot.get());
    }

    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
            SmartDashboard.putNumber("Elevator/Motor Temperature", getElevatorTemp());
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

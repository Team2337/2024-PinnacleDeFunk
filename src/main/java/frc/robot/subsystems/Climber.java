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

public class Climber extends PIDSubsystem {
    private TalonFX climbMotor = new TalonFX(54); 
    
    AnalogInput input = new AnalogInput(0);
    AnalogPotentiometer pot = new AnalogPotentiometer(input, 51.6, 1.6);

    CommandXboxController operatorJoystick;

    double climberMaxSetPoint = 10;
    double climberMinSetPoint = 1.6;

    public Climber(CommandXboxController operatorJoystick) {
        super(new PIDController(0.1, 0.0, 0.0001));
        this.operatorJoystick = operatorJoystick;
        getController().setTolerance(2.0);
        setSetpoint(pot.get());
        enable();

        var setClimbMotorToDefault = new TalonFXConfiguration();
        climbMotor.getConfigurator().apply(setClimbMotorToDefault);
        
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Voltage.PeakForwardVoltage = 8;
        leftMotorConfig.Voltage.PeakReverseVoltage = -8;

        climbMotor.setSafetyEnabled(false);
        climbMotor.getConfigurator().apply(leftMotorConfig);
        
    }

    public void setClimberSetpoint(double setPoint) {
            if (setPoint < climberMinSetPoint) {
                setPoint = climberMinSetPoint;
            } else if (setPoint > climberMaxSetPoint) {
                setPoint = climberMaxSetPoint;
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
    
    public void setClimbSpeed(double speed) {
        climbMotor.set(speed);
    }

    public void stopMotors() {
        climbMotor.stopMotor();
    }

    public double getClimbTemp() {
        return climbMotor.getDeviceTemp().getValueAsDouble();
    }

    public void getSetSetPoint() {
        setSetpoint(pot.get());
    }

    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
            SmartDashboard.putNumber("Climb/Motor Temperature", getClimbTemp());
            SmartDashboard.putNumber("Climb Position", pot.get());
            SmartDashboard.putNumber("Climb Set Point", getSetpoint());
            SmartDashboard.putBoolean("Climb at Set Point", getController().atSetpoint());
            
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
        if (output > Constants.Climber.CLIMBER_MAX_PID_SPEED) {
            output  = Constants.Climber.CLIMBER_MAX_PID_SPEED;
        } else if (output < -Constants.Climber.CLIMBER_MAX_PID_SPEED) {
            output = -Constants.Climber.CLIMBER_MAX_PID_SPEED;
        }
        setClimbSpeed(output);
        SmartDashboard.putNumber("Climb Output", output);
    }

    @Override
    protected double getMeasurement() {
       return pot.get();
    }

   
}

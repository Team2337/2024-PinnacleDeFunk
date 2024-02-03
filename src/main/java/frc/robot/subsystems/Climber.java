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
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Climber extends PIDSubsystem {
    private TalonFX climbMotorLeft = new TalonFX(54); 
    private TalonFX climbMotorRight = new TalonFX(55); 
    
    AnalogInput input = new AnalogInput(0);
    AnalogPotentiometer pot = new AnalogPotentiometer(input, 51.6, 1.6);

    public Climber() {
        super(new PIDController(0.1, 0.0, 0.0001));
        getController().setTolerance(2.0);
        setSetpoint(pot.get());
        enable();

        var setClimbMotorLeftToDefault = new TalonFXConfiguration();
        climbMotorLeft.getConfigurator().apply(setClimbMotorLeftToDefault);

        var setClimbMotorRightToDefault = new TalonFXConfiguration();
        climbMotorRight.getConfigurator().apply(setClimbMotorRightToDefault);
        
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.Voltage.PeakForwardVoltage = 8;
        leftMotorConfig.Voltage.PeakReverseVoltage = -8;

        climbMotorLeft.setSafetyEnabled(false);
        climbMotorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.Voltage.PeakForwardVoltage = 8;
        rightMotorConfig.Voltage.PeakReverseVoltage = -8;
        
        climbMotorRight.setSafetyEnabled(false);
        climbMotorRight.getConfigurator().apply(rightMotorConfig);
        
    }

    public void setClimberSetpoint(double setPoint) {
        this.setSetpoint(setPoint);
    }
    
    public void setClimbSpeed(double speed) {
        climbMotorLeft.set(speed);
        climbMotorRight.set(speed);
    }

    public void stopMotors() {
        climbMotorLeft.stopMotor();
        climbMotorRight.stopMotor();
    }

    public double getClimbLeftTemp() {
        return climbMotorLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getClimbRightTemp() {
        return climbMotorRight.getDeviceTemp().getValueAsDouble();
    }


    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
            SmartDashboard.putNumber("Climb/Left Motor Temperature", getClimbLeftTemp());
            SmartDashboard.putNumber("Climb/Right Motor Temperature", getClimbRightTemp());
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
        if (output > 0.75) {
            output  = 0.75;
        } else if (output < -0.75) {
            output = -0.75;
        }
        climbMotorLeft.set(output);
        SmartDashboard.putNumber("Climb Output", output);
    }

    @Override
    protected double getMeasurement() {
       return pot.get();
    }

   
}

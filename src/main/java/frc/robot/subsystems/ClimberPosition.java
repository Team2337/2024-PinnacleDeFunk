package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class ClimberPosition extends SubsystemBase {
    private TalonFX climbMotor = new TalonFX(54, "Upper"); 
    private final PositionVoltage voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    CommandXboxController operatorJoystick;

    double climberMaxSetPoint = 30;
    double climberMinSetPoint = -30;

    public ClimberPosition(CommandXboxController operatorJoystick) {
        this.operatorJoystick = operatorJoystick;
        
        var setClimbMotorToDefault = new TalonFXConfiguration();
        climbMotor.getConfigurator().apply(setClimbMotorToDefault);
        
        TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
        climbMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorConfig.Slot0.kP = 1;
        climbMotorConfig.Slot0.kD = 0;
        climbMotorConfig.Voltage.PeakForwardVoltage = 12;
        climbMotorConfig.Voltage.PeakReverseVoltage = -12;

        climbMotor.setSafetyEnabled(false);
        climbMotor.getConfigurator().apply(climbMotorConfig);
        climbMotor.setPosition(0);
    }

    public void setClimberSetpoint(double setPoint) {
            if (setPoint < climberMinSetPoint) {
                setPoint = climberMinSetPoint;
            } else if (setPoint > climberMaxSetPoint) {
                setPoint = climberMaxSetPoint;
            }
            //this.setSetpoint(setPoint);
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

    public double getClimberPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    public void setClimberPosition(double rotations) {
        climbMotor.setControl(voltagePosition.withPosition(rotations));
    }

    public void log() {
        if (Constants.DashboardLogging.CLIMB) {
        }
        if (Constants.DashboardLogging.TEMP) {
            SmartDashboard.putNumber("Temps/Climber Temp", getClimbTemp() );
        }
        SmartDashboard.putNumber("Climber/Get Climber Position", getClimberPosition());
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }
}

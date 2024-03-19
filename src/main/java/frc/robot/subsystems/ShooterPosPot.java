package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class ShooterPosPot extends PIDSubsystem {
    private TalonFX shootPosPotMotor = new TalonFX(50, "Upper");
    public boolean shooterAtIntake, shooterAtTrap = false;
    private Supplier<Boolean> haveNote;
    private double logDelayCounter = 0;
   
    AnalogInput input = new AnalogInput(2);
    AnalogPotentiometer pot = new AnalogPotentiometer(input, 51.6, 1.6);

    CommandXboxController operatorJoystick;
    double offset = 1.6;

    // Radial Pot Values
    double shooterPotMaxSetPoint = 15.5;
    double shooterPotMinSetPoint = 5.9;

    // String Pot Values
    // double shooterPotMinSetPoint = 3;

    public ShooterPosPot(CommandXboxController operatorJoystick, Supplier<Boolean> haveNote) {
        super(new PIDController(0.4, 0.0, 0.0001));//0.71 = P
        this.operatorJoystick = operatorJoystick;
        this.haveNote = haveNote;
        getController().setTolerance(0.1);
        setSetpoint(pot.get());
        enable();

        var setShootPosPotMotorToDefault = new TalonFXConfiguration();
        shootPosPotMotor.getConfigurator().apply(setShootPosPotMotorToDefault);

        TalonFXConfiguration shootPosPotMotorConfig = new TalonFXConfiguration();
        shootPosPotMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        shootPosPotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shootPosPotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shootPosPotMotorConfig.Voltage.PeakForwardVoltage = 12;
        shootPosPotMotorConfig.Voltage.PeakReverseVoltage = -12;

        shootPosPotMotor.setSafetyEnabled(false);
        shootPosPotMotor.getConfigurator().apply(shootPosPotMotorConfig);

        setupShuffleboard();
    }

    public void setShooterPositionPoint(double setPoint) {
            if (setPoint < shooterPotMinSetPoint) {
                setPoint = shooterPotMinSetPoint;
            } else if (setPoint > shooterPotMaxSetPoint) {
                setPoint = shooterPotMaxSetPoint;
            }
            this.setSetpoint(setPoint);
    }

    public void setShooterPosPotSpeed(double speed) {
        shootPosPotMotor.set(speed);
    }

    public void stopMotor() {
        shootPosPotMotor.stopMotor();
    }

    public double getShootPosPotTemp() {
        return shootPosPotMotor.getDeviceTemp().getValueAsDouble();
    }

    private boolean isOverheated() {
        return isMotorOverheated(shootPosPotMotor);
    }

    private boolean isMotorOverheated(TalonFX motor) {
        return motor.getDeviceTemp().getValueAsDouble() >= Constants.Global.motorShutDownTempCelcius;
    }
   
    public void getAndSetSetPoint() {
        setSetpoint(pot.get());
    }

    public void isShooterAtIntake() {
        if ((pot.get() >= (Constants.ShooterPosPot.SHOOTER_AT_PICKUP - Constants.ShooterPosPot.SHOOTERPOS_RANGE)) && (pot.get() <= (Constants.ShooterPosPot.SHOOTER_AT_PICKUP + Constants.ShooterPosPot.SHOOTERPOS_RANGE))) {
            shooterAtIntake = true;
        } else {
            shooterAtIntake = false;
        }
    }

    public void checkForNote() {
        if (!haveNote.get()) {
            setSetpoint(Constants.ShooterPosPot.SHOOTER_AT_PICKUP);
        }
    }

    public void log() {
        if (Constants.DashboardLogging.SHOOTERPOT) {
            SmartDashboard.putNumber("ShooterPosPot/ Set Point", getSetpoint());
            SmartDashboard.putBoolean("ShooterPosPot/  at Set Point", getController().atSetpoint());
            SmartDashboard.putNumber("ShooterPosPot/Motor Amp", shootPosPotMotor.getStatorCurrent().getValueAsDouble());
        }
        if (Constants.DashboardLogging.TEMP) {
            if (logDelayCounter >= Constants.Global.logDelay) {
                SmartDashboard.putNumber("ShooterPosPot/ Motor Temperature", getShootPosPotTemp());
                logDelayCounter = 0;
            }
        }
        logDelayCounter++;
    }

    public void setupShuffleboard() {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;
      
      systemsCheck.addDouble("Shooter Position", () -> pot.get())
        .withPosition(SystemsCheckPositions.SHOOTER_POSITION.x, SystemsCheckPositions.SHOOTER_POSITION.y)
        .withSize(2, 1);
    
    }

    public void initialize() {
        setSetpoint(pot.get());
    }

    @Override
    public void periodic() {
        super.periodic();
        isShooterAtIntake();
        checkForNote();
        log();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (output > Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED) {
            output  = Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED;
        } else if (output < -Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED) {
            output = -Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED;
        }
        setShooterPosPotSpeed(output);
        // if (getSetpoint() > 30) {
        //     stopMotor();
        // }
        //SmartDashboard.putNumber("ShooterPosPot/Output", output);
    }

    @Override
    protected double getMeasurement() {
       return pot.get();
    }

   
}

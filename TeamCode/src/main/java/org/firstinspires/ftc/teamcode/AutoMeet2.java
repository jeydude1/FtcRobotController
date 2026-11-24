package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Auto Meet 2", group = "Autonomous")
public class AutoMeet2 extends LinearOpMode {

    // DRIVE CONSTANTS
    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double CORRECTION_VALUE = 0.92;
    private static final double STRAFE_MULTIPLIER = 1.1;
    private static final double DRIVE_SPEED = 0.5;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO /
            (Math.PI * WHEEL_DIAMETER_INCHES))
            * CORRECTION_VALUE;

    // HARDWARE
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx intake;
    private DcMotorEx shooter;
    private DcMotorEx spinner;

    private VoltageSensor batteryVoltageSensor;


    // SHOOTER VELOCITY CONTROLLER
    // Same settings as TeleOp

    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double MAX_ACHIEVABLE_RPM = 5200.0;

    private static final double kV = 1.0 / MAX_ACHIEVABLE_RPM;
    private static final double kS = 0.04;
    private static final double kP = 0.0012;

    private static final double TARGET_RPM = 2200.0;
    private static final double RPM_TOLERANCE = 90.0;

    private static final double FEEDER_POWER = 0.85;

    private static final double SHOOTER_READY_TIMEOUT = 3000;
    private static final double SHOOTING_TIME = 4000;

    private boolean shooterEnabled = false;
    private boolean atTargetSpeed = false;

    private double currentRPM = 0;
    private double rpmError = 0;
    private double shooterPower = 0;

    // TIMERS
    private ElapsedTime runtime = new ElapsedTime();

    private boolean shootingTimerStarted = false;

    // AUTONOMOUS STATES
    private enum AutoState {
        START,
        MOVE_FORWARD,
        SHOOT,
        TURN_TO_RIGHT,
        FORWARD,
        PARK,
        STOP,
        DONE
    }

    private AutoState state = AutoState.START;
    private boolean moveStarted = false;

    // MAIN AUTONOMOUS
    @Override
    public void runOpMode() {

        initDrive();

        telemetry.addLine("Auto Meet 2 Ready");
        telemetry.addLine("Velocity Shooter Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAllMotors();
            return;
        }

        runtime.reset();

        while (opModeIsActive() && state != AutoState.DONE) {

            boolean feedRequested = false;

            switch (state) {

                case START:

                    shooterEnabled = true;

                    if (atTargetSpeed ||
                            runtime.milliseconds() > SHOOTER_READY_TIMEOUT) {

                        state = AutoState.MOVE_FORWARD;
                        runtime.reset();
                    }

                    break;

                case MOVE_FORWARD:

                    if (!moveStarted) {
                        drive(-7, DRIVE_SPEED);
                        moveStarted = true;
                    }

                    if (driveCompleted()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.SHOOT;
                        runtime.reset();
                        shootingTimerStarted = false;
                    }

                    break;

                case SHOOT:

                    feedRequested = true;

                    if (atTargetSpeed && !shootingTimerStarted) {
                        runtime.reset();
                        shootingTimerStarted = true;
                    }

                    if (shootingTimerStarted &&
                            runtime.milliseconds() > SHOOTING_TIME) {

                        state = AutoState.TURN_TO_RIGHT;
                        runtime.reset();
                    }

                    break;

                case TURN_TO_RIGHT:

                    shooterEnabled = false;

                    if (!moveStarted) {
                        strafe(27, DRIVE_SPEED);
                        moveStarted = true;
                    }

                    if (driveCompleted()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.FORWARD;
                        runtime.reset();
                    }

                    break;

                case FORWARD:

                    if (!moveStarted) {
                        drive(-60, DRIVE_SPEED);
                        moveStarted = true;
                    }

                    if (driveCompleted()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.PARK;
                        runtime.reset();
                    }

                    break;

                case PARK:

                    if (!moveStarted) {
                        strafe(13, DRIVE_SPEED);
                        moveStarted = true;
                    }

                    if (driveCompleted()) {
                        stopDrive();
                        moveStarted = false;
                        state = AutoState.STOP;
                        runtime.reset();
                    }

                    break;

                case STOP:

                    if (!moveStarted) {
                        strafe(-2, DRIVE_SPEED);
                        moveStarted = true;
                    }

                    if (driveCompleted()) {
                        stopDrive();
                        moveStarted = false;
                        shooterEnabled = false;
                        stopAllMotors();
                        state = AutoState.DONE;
                    }

                    break;

                case DONE:
                    break;
            }

            updateShooter(feedRequested);

            telemetry.addLine("===== AUTONOMOUS =====");
            telemetry.addData("State", state);

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("RPM Error", "%.0f", rpmError);
            telemetry.addData("Shooter Power", "%.3f", shooterPower);

            telemetry.addData(
                    "Shooter Status",
                    atTargetSpeed ? "READY TO FIRE" : "SPINNING UP / RECOVERING"
            );

            telemetry.addData(
                    "Feeder",
                    feedRequested && shooterEnabled && atTargetSpeed
                            ? "FEEDING"
                            : "OFF"
            );

            telemetry.addData(
                    "Battery Voltage",
                    "%.2f V",
                    batteryVoltageSensor.getVoltage()
            );

            telemetry.addLine("===== DRIVE =====");

            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());

            telemetry.update();
        }

        stopAllMotors();

        telemetry.addLine("Path Complete");
        telemetry.update();

        sleep(1000);
    }

    // SHOOTER VELOCITY CONTROLLER
    private void updateShooter(boolean feedRequested) {

        double currentTicksPerSec = shooter.getVelocity();

        currentRPM =
                (currentTicksPerSec * 60.0) /
                SHOOTER_TICKS_PER_REV;

        rpmError = TARGET_RPM - currentRPM;

        atTargetSpeed =
                Math.abs(rpmError) <= RPM_TOLERANCE;

        shooterPower = 0;

        if (shooterEnabled) {

            double feedforward =
                    (TARGET_RPM * kV) + kS;

            double proportional =
                    rpmError * kP;

            double desired12VPower =
                    feedforward + proportional;

            double currentVoltage =
                    batteryVoltageSensor.getVoltage();

            double voltageScale =
                    currentVoltage > 8.0
                            ? 12.0 / currentVoltage
                            : 1.0;

            shooterPower = Range.clip(
                    desired12VPower * voltageScale,
                    0.0,
                    1.0
            );
        }

        shooter.setPower(shooterPower);

        // RPM-GATED FEEDER
        if (feedRequested && shooterEnabled && atTargetSpeed) {
            spinner.setPower(FEEDER_POWER);
            intake.setPower(1.0);
        } else {
            spinner.setPower(0.0);
            intake.setPower(0.0);
        }
    }

    // HARDWARE INITIALIZATION
    private void initDrive() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        backRight = hardwareMap.get(DcMotorEx.class, "backright");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");

        batteryVoltageSensor =
                hardwareMap.voltageSensor.iterator().next();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{
                frontLeft, frontRight, backLeft, backRight
        }) {

            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );

            motor.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER
            );

            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );
        }

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);
        shooter.setPower(0);
        spinner.setPower(0);

    }

    // DRIVE METHODS
    private void drive(double inches, double power) {

        int ticks = (int) (inches * TICKS_PER_INCH);

        setTargets(ticks, ticks, ticks, ticks);

        runToPosition(power);
    }

    private void strafe(double inches, double power) {

        int ticks = (int) (
                inches * TICKS_PER_INCH * STRAFE_MULTIPLIER
        );

        setTargets(ticks, -ticks, -ticks, ticks);

        runToPosition(power);
    }

    private void setTargets(
            int flT,
            int frT,
            int blT,
            int brT) {

        frontLeft.setTargetPosition(
                frontLeft.getCurrentPosition() + flT
        );

        frontRight.setTargetPosition(
                frontRight.getCurrentPosition() + frT
        );

        backLeft.setTargetPosition(
                backLeft.getCurrentPosition() + blT
        );

        backRight.setTargetPosition(
                backRight.getCurrentPosition() + brT
        );
    }

    private void runToPosition(double power) {

        DcMotorEx[] motors = {
                frontLeft,
                frontRight,
                backLeft,
                backRight
        };

        for (DcMotorEx motor : motors) {

            if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            motor.setPower(Math.abs(power));
        }
    }

    private boolean driveCompleted() {

        return !frontLeft.isBusy()
                && !frontRight.isBusy()
                && !backLeft.isBusy()
                && !backRight.isBusy();
    }

    private void stopDrive() {

        for (DcMotorEx motor : new DcMotorEx[]{
                frontLeft, frontRight, backLeft, backRight
        }) {

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    // STOP ALL MOTORS
    private void stopAllMotors() {

        stopDrive();

        if (shooter != null) shooter.setPower(0);
        if (intake != null) intake.setPower(0);
        if (spinner != null) spinner.setPower(0);
    }
}

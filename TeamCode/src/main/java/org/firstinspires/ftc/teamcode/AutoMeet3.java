package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Autonomous(name = "Auto Meet 3", group = "Autonomous")
public class AutoMeet3 extends LinearOpMode {

    // DRIVE CONSTANTS
    private static final double TICKS_PER_REV = 384.5;   // goBILDA 5203 13.7:1 (435 RPM) encoder ticks per output rev
    private static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;   // 104 mm mecanum wheel = 4.094 in
    private static final double GEAR_RATIO = 1.0;
    private static final double CORRECTION_VALUE = 0.92;
    private static final double STRAFE_MULTIPLIER = 1.1;
    private static final double SLOW_DRIVE_SPEED = 0.5;
    private static final double DRIVE_SPEED = 0.80;

    private static final int DRIVE_TOLERANCE = 20;

    // SPEED RAMP (smooth starts and stops)
    // Each move starts at MIN_DRIVE_POWER, speeds up over RAMP_UP_TICKS,
    // and slows back down over the last RAMP_DOWN_TICKS before the target.
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double RAMP_UP_TICKS = 150;     // ~5.5 in
    private static final double RAMP_DOWN_TICKS = 300;   // ~11 in

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

    private List<LynxModule> hubs;

    // Current move, used by the speed ramp
    private DcMotorEx[] driveMotors;
    private final int[] moveStartPositions = new int[4];
    private double moveMaxPower = 0;


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
    private static final double SHOOTING_TIME = 3000;

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

        telemetry.addLine("Auto Meet 3 Ready");
        telemetry.addLine("Velocity Shooter Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAllMotors();
            return;
        }

        runtime.reset();

        while (opModeIsActive() && state != AutoState.DONE) {

            // Fresh sensor readings for this loop (one bulk read per hub)
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

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
                        drive(-9, SLOW_DRIVE_SPEED);
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
                        strafe(36, DRIVE_SPEED);
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
                        drive(-85, DRIVE_SPEED);
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
                        strafe(23, DRIVE_SPEED);
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

            // Adjust drive speed for the move in progress
            if (moveStarted) {
                updateDriveRamp();
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

        // Read all motor encoders in one request per hub instead of one
        // request per call, so the loop (and the speed ramp) runs faster.
        // The cache is cleared at the top of every loop.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        backRight = hardwareMap.get(DcMotorEx.class, "backright");

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

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

        // Remember where this move started, for the speed ramp
        for (int i = 0; i < 4; i++) {
            moveStartPositions[i] = driveMotors[i].getCurrentPosition();
        }

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

        // The move starts slow; updateDriveRamp() speeds it up each loop
        moveMaxPower = Math.abs(power);

        for (DcMotorEx motor : driveMotors) {

            if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            motor.setPower(Math.min(MIN_DRIVE_POWER, moveMaxPower));
        }
    }

    // Called every loop while a move is running.
    // Power climbs from MIN_DRIVE_POWER as the robot leaves the start and
    // drops back to MIN_DRIVE_POWER as it nears the target, so the robot
    // doesn't jerk when starting or skid when stopping.
    private void updateDriveRamp() {

        double traveled = 0;
        double remaining = 0;

        for (int i = 0; i < 4; i++) {
            int pos = driveMotors[i].getCurrentPosition();
            traveled += Math.abs(pos - moveStartPositions[i]);
            remaining += Math.abs(driveMotors[i].getTargetPosition() - pos);
        }

        // Average across the four wheels
        traveled /= 4.0;
        remaining /= 4.0;

        double minPower = Math.min(MIN_DRIVE_POWER, moveMaxPower);
        double range = moveMaxPower - minPower;

        double rampUp = minPower + (traveled / RAMP_UP_TICKS) * range;
        double rampDown = minPower + (remaining / RAMP_DOWN_TICKS) * range;

        double power = Range.clip(
                Math.min(rampUp, rampDown),
                minPower,
                moveMaxPower
        );

        for (DcMotorEx motor : driveMotors) {
            motor.setPower(power);
        }
    }

    private boolean driveCompleted() {

        return Math.abs(frontLeft.getTargetPosition()
                - frontLeft.getCurrentPosition()) <= DRIVE_TOLERANCE

                && Math.abs(frontRight.getTargetPosition()
                - frontRight.getCurrentPosition()) <= DRIVE_TOLERANCE

                && Math.abs(backLeft.getTargetPosition()
                - backLeft.getCurrentPosition()) <= DRIVE_TOLERANCE

                && Math.abs(backRight.getTargetPosition()
                - backRight.getCurrentPosition()) <= DRIVE_TOLERANCE;
    }
    private void stopDrive() {

        for (DcMotorEx motor : new DcMotorEx[]{
                frontLeft, frontRight, backLeft, backRight
        }) {

            // Stay in RUN_TO_POSITION between moves; the next move sets
            // new targets and power.
            motor.setPower(0);
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

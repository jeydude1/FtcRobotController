
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Meet 2 TeleOp", group = "TeleOp")
public class Meet_2_TeleOp extends LinearOpMode {

    // ================= HARDWARE =================

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotor spinner;

    private Servo blocker;
    private VoltageSensor batteryVoltageSensor;

    private static final double BLOCKER_DOWN = 0.2;
    private static final double BLOCKER_UP = 0.35;

    // ================= SHOOTER CONSTANTS =================

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_ACHIEVABLE_RPM = 5200.0;

    private double targetRPM = 2200.0;
    private double rpmTolerance = 90.0;

    private static final double kV = 1.0 / MAX_ACHIEVABLE_RPM;
    private static final double kS = 0.04;
    private static final double kP = 0.0012;

    private static final double FEEDER_POWER = 0.85;

    private boolean shooterEnabled = false;

    // ================= DRIVE =================

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ================= HARDWARE INITIALIZATION =================

        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        blocker = hardwareMap.get(Servo.class, "blocker");

        batteryVoltageSensor =
                hardwareMap.voltageSensor.iterator().next();

        // ================= MOTOR DIRECTIONS =================

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // ================= MOTOR MODES =================

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter uses direct power control.
        // Encoder velocity is still available through getVelocity().

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ================= ZERO POWER BEHAVIOR =================

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Never actively brake the flywheel when power is zero.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Meet 2 TeleOp Ready");
        telemetry.addLine("Shooter RPM Controller Ready");
        telemetry.update();

        waitForStart();

        // ================= MAIN LOOP =================

        while (opModeIsActive()) {

            // =====================================================
            // MECANUM DRIVE
            // =====================================================

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(
                                    Math.abs(frontLeftPower),
                                    Math.abs(backLeftPower)
                            ),
                            Math.max(
                                    Math.abs(frontRightPower),
                                    Math.abs(backRightPower)
                            )
                    )
            );

            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;

            // Slow mode
            double speed = gamepad1.left_bumper ? 0.4 : 1.0;

            frontLeft.setPower(frontLeftPower * speed);
            backLeft.setPower(backLeftPower * speed);
            frontRight.setPower(frontRightPower * speed);
            backRight.setPower(backRightPower * speed);

            // =====================================================
            // SHOOTER BUTTON CONTROLS
            // =====================================================

            // Toggle shooter ON/OFF
            if (gamepad2.leftBumperWasPressed()) {
                shooterEnabled = !shooterEnabled;
            }

            // Increase target RPM
            if (gamepad2.dpad_up) {
                targetRPM += 50;
            }

            // Decrease target RPM
            if (gamepad2.dpad_down) {
                targetRPM -= 50;
            }

            // Keep target within a reasonable tuning range
            targetRPM = Range.clip(targetRPM, 0, MAX_ACHIEVABLE_RPM);

            // Stop shooter
            if (gamepad2.leftTriggerWasPressed()) {
                shooterEnabled = false;
                shooter.setPower(0);
                spinner.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_UP);
            }

            // =====================================================
            // READ SHOOTER RPM
            // =====================================================

            double currentTicksPerSec = shooter.getVelocity();

            double currentRPM =
                    (currentTicksPerSec * 60.0) / TICKS_PER_REV;

            double rpmError = targetRPM - currentRPM;

            boolean atTargetSpeed =
                    Math.abs(rpmError) <= rpmTolerance;

            // =====================================================
            // SHOOTER FEEDFORWARD + P CONTROLLER
            // =====================================================

            double shooterPower = 0.0;

            if (shooterEnabled) {

                // Feedforward
                double feedforward =
                        (targetRPM * kV) + kS;

                // Proportional recovery
                double proportional =
                        rpmError * kP;

                double desired12VPower =
                        feedforward + proportional;

                // Battery voltage compensation
                double currentVoltage =
                        batteryVoltageSensor.getVoltage();

                double voltageScale =
                        (currentVoltage > 8.0)
                                ? (12.0 / currentVoltage)
                                : 1.0;

                shooterPower = Range.clip(
                        desired12VPower * voltageScale,
                        0.0,
                        1.0
                );
            }

            // Reverse shooter while right trigger is held
            if (gamepad2.right_trigger > 0.1 && !shooterEnabled) {
                shooter.setPower(-0.5);
            } else {
                shooter.setPower(shooterPower);
            }

            // =====================================================
            // INTAKE CONTROLS
            // =====================================================

            // Intake and spinner forward
            if (gamepad1.rightBumperWasPressed()) {
                intake.setPower(1.0);
                spinner.setPower(1.0);
            } else if (gamepad1.rightBumperWasReleased()) {
                intake.setPower(0.0);
                spinner.setPower(0.0);
            }

            // Reverse intake
            if (gamepad1.rightTriggerWasPressed()) {
                intake.setPower(-1.0);
            } else if (gamepad1.rightTriggerWasReleased()) {
                intake.setPower(0.0);
            }
            // Reverse shooter, spinner, and intake
            if (gamepad2.backWasPressed()) {
                spinner.setPower(-1.0);
                intake.setPower(-1.0);
                shooter.setPower(-1.0);

                sleep(200);

                stopAllMotors();
                shooterEnabled = false;
            }
            

            // Reverse spinner
            if (gamepad2.rightBumperWasPressed()) {
                spinner.setPower(-1.0);
            } else if (gamepad2.rightBumperWasReleased()) {
                spinner.setPower(0.0);
            }

            // =====================================================
            // RPM-GATED SHOOTER FEEDER
            // =====================================================

            // Hold GAMEPAD 2 B to feed.
            // Feeder only runs when shooter reaches target RPM.

            if (gamepad2.b) {

                if (shooterEnabled && atTargetSpeed) {

                    spinner.setPower(FEEDER_POWER);
                    intake.setPower(1.0);

                } else {

                    spinner.setPower(0.0);
                    intake.setPower(0.0);
                }
            }

            

            // =====================================================
            // SHOOTING PREP / REVERSE MOTORS
            // =====================================================

            // Press GAMEPAD 1 BACK to reverse motors briefly.
            // Moved from left bumper to avoid conflicting with slow mode.

            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addLine("===== DRIVE =====");

            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("BR", "%.2f", backRightPower);

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData(
                    "Shooter Status",
                    shooterEnabled
                            ? (atTargetSpeed
                                ? "READY TO FIRE!"
                                : "SPINNING UP / RECOVERING")
                            : "OFF"
            );

            telemetry.addData(
                    "Target RPM",
                    "%.0f RPM", targetRPM
            );

            telemetry.addData(
                    "Current RPM",
                    "%.0f RPM", currentRPM
            );

            telemetry.addData(
                    "RPM Error",
                    "%.0f RPM", rpmError
            );

            telemetry.addData(
                    "Shooter Power",
                    "%.3f", shooterPower
            );

            telemetry.addData(
                    "Battery Voltage",
                    "%.2f V",
                    batteryVoltageSensor.getVoltage()
            );

            telemetry.addData(
                    "Feeder",
                    gamepad2.b
                            ? (shooterEnabled && atTargetSpeed
                                ? "FEEDING"
                                : "WAITING FOR RPM")
                            : "OFF"
            );

            telemetry.update();
        }
    }

    // =====================================================
    // STOP MOTORS
    // =====================================================

    private void stopAllMotors() {

        if (shooter != null) shooter.setPower(0);
        if (intake != null) intake.setPower(0);
        if (spinner != null) spinner.setPower(0);
    }
}

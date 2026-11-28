
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Meet 3 RED Distance RPM", group = "TeleOp")

public class Meet3TeleOpLimelight extends LinearOpMode {

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

    private Limelight3A limelight;

    // ================= SERVO =================

    private static final double BLOCKER_UP = 0.5;
    private static final double BLOCKER_DOWN = 0.2;

    // ================= SHOOTER =================

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_ACHIEVABLE_RPM = 5200.0;

    private double targetRPM = 2250.0;
    private double rpmTolerance = 90.0;

    private static final double kV = 1.0 / MAX_ACHIEVABLE_RPM;
    private static final double kS = 0.04;
    private static final double kP = 0.0012;

    private static final double FEEDER_POWER = 0.85;

    private boolean shooterEnabled = false;

    // Time to let the blocker servo reach BLOCKER_UP before feeding
    private static final double BLOCKER_RAISE_MS = 1000.0;
    private ElapsedTime blockerTimer = new ElapsedTime();
    private boolean autoRpm = true;

    // ================= RED CELL TAGS =================

    private static final int[] RED_FAR_CELL_TAGS = {
            30, 31, 32, 33
    };

    private static final int[] RED_AUDIENCE_CELL_TAGS = {
            34, 35, 36, 37
    };

    private boolean farCell = true;

    private int[] targetTags = RED_FAR_CELL_TAGS;

    // ================= DISTANCE =================

    private static final double CAMERA_HEIGHT_IN = 16.25;
    private static final double CAMERA_ANGLE_DEG = 22.0;
    private static final double GOAL_TAG_HEIGHT_IN = 50;

    // ================= RPM TABLE =================

    private static final double[] RPM_TABLE_DIST_IN = {
            50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
    };

    private static final double[] RPM_TABLE_RPM = {
            2250, 2280, 2310, 2340, 2370, 2400, 2430, 2460, 2490, 2530, 2570
    };

    // ================= INITIALIZATION =================

    @Override
    public void runOpMode() {

        // Hardware mapping

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Motor directions

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor modes

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake behavior

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker.setPosition(BLOCKER_DOWN);

        // ================= INIT: SELECT RED CELL =================

        while (opModeInInit()) {

            if (gamepad1.dpad_up) {
                farCell = true;
            }

            if (gamepad1.dpad_down) {
                farCell = false;
            }

            selectTargetCell();

            GoalTarget preview = findGoal();

            telemetry.addLine("MEET 3 RED DISTANCE RPM");
            telemetry.addLine("RED ALLIANCE ONLY");
            telemetry.addLine("DPAD UP = FAR CELL");
            telemetry.addLine("DPAD DOWN = AUDIENCE CELL");

            telemetry.addData("Target", targetName());
            telemetry.addData("Tags Seen", preview.tagCount);
            telemetry.addData("Limelight Connected",
                    limelight.isConnected());

            telemetry.update();
        }

        // ================= MAIN LOOP =================

        while (opModeIsActive()) {

            GoalTarget goal = findGoal();

            // ================= DRIVE =================

            double speed = gamepad1.left_bumper ? 0.4 : 1.0;

            double y = -gamepad1.left_stick_y * speed;
            double x = gamepad1.left_stick_x * speed;
            double rx = gamepad1.right_stick_x * speed;

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))
            ));

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // ================= SHOOTER CONTROLS =================

            if (gamepad2.leftBumperWasPressed()) {
                shooterEnabled = !shooterEnabled;
            }

            if (gamepad2.yWasPressed()) {
                autoRpm = !autoRpm;
            }

            if (gamepad2.dpadUpWasPressed()) {
                targetRPM += 50;
                autoRpm = false;
            }

            if (gamepad2.dpadDownWasPressed()) {
                targetRPM -= 50;
                autoRpm = false;
            }

            if (gamepad2.leftTriggerWasPressed()) {
                shooterEnabled = false;
                shooter.setPower(0);
                spinner.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN);
            }

            // ================= AUTO RPM =================

            if (autoRpm && goal.visible && goal.distanceIn > 0) {
                targetRPM = rpmForDistance(goal.distanceIn);
            }

            targetRPM = Range.clip(
                    targetRPM, 0, MAX_ACHIEVABLE_RPM);

            // ================= SHOOTER RPM =================

            double currentRPM =
                    (shooter.getVelocity() * 60.0) / TICKS_PER_REV;

            double rpmError = targetRPM - currentRPM;

            boolean atTargetSpeed =
                    Math.abs(rpmError) <= rpmTolerance;

            double shooterPower = 0;

            if (shooterEnabled) {

                double feedforward = targetRPM * kV + kS;
                double proportional = rpmError * kP;

                double desiredPower = feedforward + proportional;

                double voltage = batteryVoltageSensor.getVoltage();

                double voltageScale =
                        voltage > 8.0 ? 12.0 / voltage : 1.0;

                shooterPower = Range.clip(
                        desiredPower * voltageScale, 0, 1);
            }

            // ================= INTAKE CONTROLS =================

            if (gamepad1.rightBumperWasPressed()) {
                intake.setPower(1.0);
                spinner.setPower(1.0);
            }

            if (gamepad1.rightBumperWasReleased()) {
                intake.setPower(0);
                spinner.setPower(0);
            }

            if (gamepad1.rightTriggerWasPressed()) {
                intake.setPower(-1.0);
            }

            if (gamepad1.rightTriggerWasReleased()) {
                intake.setPower(0);
            }

            // ================= SPINNER REVERSE =================

            if (gamepad2.rightBumperWasPressed()) {
                spinner.setPower(-1.0);
            }

            if (gamepad2.rightBumperWasReleased()) {
                spinner.setPower(0);
            }

            // ================= UNJAM =================

            if (gamepad2.backWasPressed()) {

                spinner.setPower(-1.0);
                intake.setPower(-1.0);
                shooter.setPower(-1.0);

                sleep(200);

                stopAllMotors();

                shooterEnabled = false;
            }

            // ================= SHOOTER POWER =================

            // Normal shooter power, unless unjam or stop was requested.
            if (shooterEnabled) {
                shooter.setPower(shooterPower);
            } else if (gamepad2.right_trigger > 0.1) {
                shooter.setPower(-0.5);
            } else {
                shooter.setPower(0);
            }

            // ================= FEEDER =================

            if (gamepad2.bWasPressed()) {
                blocker.setPosition(BLOCKER_UP);
                blockerTimer.reset();
            }

            boolean blockerRaised =
                    blockerTimer.milliseconds() >= BLOCKER_RAISE_MS;

            if (gamepad2.b && blockerRaised) {

                if (shooterEnabled && atTargetSpeed) {

                    spinner.setPower(FEEDER_POWER);
                    intake.setPower(1.0);
                }
                else {

                    spinner.setPower(0.0);
                    intake.setPower(0.0);
                }
            }

            // ================= TELEMETRY =================

            telemetry.addLine("===== RED LIMELIGHT =====");

            telemetry.addData("Connected",
                    limelight.isConnected());

            telemetry.addData("Target", targetName());

            telemetry.addData("Tags Seen", goal.tagCount);

            telemetry.addData("Goal Visible", goal.visible);

            telemetry.addData("Distance",
                    "%.1f inches", goal.distanceIn);

            telemetry.addData("Auto RPM",
                    autoRpm ? "ON" : "OFF");

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData("Shooter",
                    shooterEnabled ? "ON" : "OFF");

            telemetry.addData("Target RPM",
                    "%.0f", targetRPM);

            telemetry.addData("Current RPM",
                    "%.0f", currentRPM);

            telemetry.addData("Shooter Power",
                    "%.3f", shooterPower);

            telemetry.addData("At Target Speed",
                    atTargetSpeed);

            telemetry.addData("Battery",
                    "%.2f V", batteryVoltageSensor.getVoltage());

            telemetry.addData("Feeder",
                    gamepad2.b
                            ? (shooterEnabled && atTargetSpeed
                            ? "FEEDING" : "WAITING FOR RPM")
                            : "OFF");

            telemetry.update();
        }

        stopAllMotors();
        limelight.stop();
    }

    // ================= LIMELIGHT HELPERS =================

    private static class GoalTarget {
        boolean visible = false;
        int tagCount = 0;
        double ty = 0;
        double distanceIn = 0;
    }

    private void selectTargetCell() {

        targetTags = farCell
                ? RED_FAR_CELL_TAGS
                : RED_AUDIENCE_CELL_TAGS;
    }

    private String targetName() {

        return farCell
                ? "RED FAR CELL (30-33)"
                : "RED AUDIENCE CELL (34-37)";
    }

    private boolean isTargetTag(int id) {

        for (int tag : targetTags) {
            if (tag == id) return true;
        }

        return false;
    }

    private GoalTarget findGoal() {

        GoalTarget goal = new GoalTarget();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return goal;
        }

        double tySum = 0;

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {

            if (isTargetTag(tag.getFiducialId())) {
                tySum += tag.getTargetYDegrees();
                goal.tagCount++;
            }
        }

        if (goal.tagCount > 0) {
            goal.visible = true;
            goal.ty = tySum / goal.tagCount;
            goal.distanceIn = distanceToGoal(goal.ty);
        }

        return goal;
    }

    // ================= DISTANCE =================

    private double distanceToGoal(double tyDeg) {

        double angleRad = Math.toRadians(
                CAMERA_ANGLE_DEG + tyDeg);

        if (angleRad <= 0.01) return 0;

        return (GOAL_TAG_HEIGHT_IN - CAMERA_HEIGHT_IN)
                / Math.tan(angleRad);
    }

    // ================= RPM TABLE =================

    private double rpmForDistance(double distanceIn) {

        if (distanceIn <= RPM_TABLE_DIST_IN[0]) {
            return RPM_TABLE_RPM[0];
        }

        int last = RPM_TABLE_DIST_IN.length - 1;

        if (distanceIn >= RPM_TABLE_DIST_IN[last]) {
            return RPM_TABLE_RPM[last];
        }

        for (int i = 0; i < last; i++) {

            double d0 = RPM_TABLE_DIST_IN[i];
            double d1 = RPM_TABLE_DIST_IN[i + 1];

            if (distanceIn <= d1) {

                double t = (distanceIn - d0) / (d1 - d0);

                return RPM_TABLE_RPM[i]
                        + t * (RPM_TABLE_RPM[i + 1]
                        - RPM_TABLE_RPM[i]);
            }
        }

        return RPM_TABLE_RPM[last];
    }

    // ================= STOP MOTORS =================

    private void stopAllMotors() {

        if (shooter != null) shooter.setPower(0);
        if (intake != null) intake.setPower(0);
        if (spinner != null) spinner.setPower(0);
    }
}

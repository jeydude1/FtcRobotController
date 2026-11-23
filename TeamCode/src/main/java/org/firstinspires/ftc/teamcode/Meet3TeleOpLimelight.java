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
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Meet 3 TeleOp with Limelight 3A auto-aim (BIOBUZZ 2026-27).
 *
 * Same controls as Meet 2 TeleOp, plus:
 *   INIT:       Gamepad 1 X = BLUE alliance, Gamepad 1 B = RED alliance
 *               Gamepad 1 D-pad UP = far CELL (opposite the audience),
 *               Gamepad 1 D-pad DOWN = audience-side CELL
 *   Gamepad 1   LT (hold) = auto-aim: robot turns to face the chosen CELL.
 *                           You still drive/strafe with the left stick.
 *   Gamepad 2   Y (press) = auto RPM on/off (RPM from distance to the goal).
 *                           Pressing D-pad up/down turns auto RPM off.
 *
 * Needs a Limelight named "limelight" in the robot configuration, running an
 * AprilTag pipeline in slot 0. Keep "Meet 2 TeleOp" as a backup that works
 * without the Limelight.
 */
@TeleOp(name = "Meet 3 TeleOp (Limelight)", group = "TeleOp")
public class Meet3_TeleOp_Limelight extends LinearOpMode {

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

    private static final double BLOCKER_DOWN = 0.2;
    private static final double BLOCKER_UP = 0.35;

    // ================= SHOOTER CONSTANTS (same as Meet 2) =================

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_ACHIEVABLE_RPM = 5200.0;

    private double targetRPM = 2200.0;
    private double rpmTolerance = 90.0;

    private static final double kV = 1.0 / MAX_ACHIEVABLE_RPM;
    private static final double kS = 0.04;
    private static final double kP = 0.0012;

    private static final double FEEDER_POWER = 0.85;

    private boolean shooterEnabled = false;

    // ================= LIMELIGHT =================

    // BIOBUZZ CELL AprilTag IDs (4 tags per CELL).
    // Confirm against the current BIOBUZZ Competition Manual / Team Updates.
    private static final int[] RED_FAR_CELL_TAGS = {30, 31, 32, 33};       // side opposite the audience
    private static final int[] RED_AUDIENCE_CELL_TAGS = {34, 35, 36, 37};  // audience side
    private static final int[] BLUE_AUDIENCE_CELL_TAGS = {38, 39, 40, 41}; // audience side
    private static final int[] BLUE_FAR_CELL_TAGS = {42, 43, 44, 45};      // side opposite the audience

    // Chosen in INIT
    private boolean blueAlliance = true;
    private boolean farCell = true;
    private int[] targetTags = BLUE_FAR_CELL_TAGS;

    // Auto-aim turning (tune on the robot)
    private static final double AIM_KP = 0.02;          // turn power per degree off target
    private static final double AIM_MAX_TURN = 0.40;    // fastest auto-aim turn
    private static final double AIM_MIN_TURN = 0.06;    // smallest turn that still moves the robot
    private static final double AIM_TOLERANCE_DEG = 1.5; // "aimed" when this close

    // Distance from the camera to the goal tag, from the tag's vertical angle.
    // MEASURE THESE on your robot and field.
    private static final double CAMERA_HEIGHT_IN = 10.0;    // lens height above the floor
    private static final double CAMERA_ANGLE_DEG = 20.0;    // camera tilt up from level
    private static final double GOAL_TAG_HEIGHT_IN = 29.5;  // PLACEHOLDER: average height of the CELL tags above the floor

    // Auto RPM table: distance (in) -> shooter RPM.
    // PLACEHOLDER VALUES: shoot from each distance and record the RPM that scores.
    private static final double[] RPM_TABLE_DIST_IN = {24, 36, 48, 60, 72, 96};
    private static final double[] RPM_TABLE_RPM = {2000, 2100, 2200, 2350, 2500, 2800};

    private boolean autoRpm = false;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);   // slot 0 = AprilTag pipeline
        limelight.start();             // without start(), results are always null

        // ================= MOTOR DIRECTIONS =================

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // ================= MOTOR MODES =================

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ================= ZERO POWER BEHAVIOR =================

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================= INIT: PICK ALLIANCE =================

        while (opModeInInit()) {
            if (gamepad1.x) blueAlliance = true;
            if (gamepad1.b) blueAlliance = false;
            if (gamepad1.dpad_up) farCell = true;
            if (gamepad1.dpad_down) farCell = false;
            selectTargetCell();

            GoalTarget preview = findGoal();

            telemetry.addLine("Meet 3 TeleOp (Limelight) Ready");
            telemetry.addLine("Gamepad 1: X = BLUE, B = RED");
            telemetry.addLine("Gamepad 1: D-pad UP = far CELL, DOWN = audience CELL");
            telemetry.addData("Target", targetName());
            telemetry.addData("CELL tags seen", preview.tagCount);
            telemetry.update();
        }

        // ================= MAIN LOOP =================

        while (opModeIsActive()) {

            // =====================================================
            // LIMELIGHT: FIND OUR GOAL
            // =====================================================

            GoalTarget goal = findGoal();

            // =====================================================
            // MECANUM DRIVE (+ AUTO-AIM ON GAMEPAD 1 LT)
            // =====================================================

            double speed = gamepad1.left_bumper ? 0.4 : 1.0;   // slow mode

            double y = -gamepad1.left_stick_y * speed;
            double x = gamepad1.left_stick_x * speed;
            double rx = gamepad1.right_stick_x * speed;

            boolean aiming = gamepad1.left_trigger > 0.2 && goal.visible;
            boolean aimed = goal.visible && Math.abs(goal.tx) <= AIM_TOLERANCE_DEG;

            if (aiming) {
                // Turn toward the tag. tx > 0 means the tag is to the right.
                // If the robot turns AWAY from the goal, flip the sign here.
                if (aimed) {
                    rx = 0;
                } else {
                    double turn = goal.tx * AIM_KP;
                    turn = Math.copySign(
                            Math.max(Math.abs(turn), AIM_MIN_TURN), turn);
                    rx = Range.clip(turn, -AIM_MAX_TURN, AIM_MAX_TURN);
                }
            }

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
            ));

            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);

            // =====================================================
            // SHOOTER BUTTON CONTROLS
            // =====================================================

            // Toggle shooter ON/OFF
            if (gamepad2.leftBumperWasPressed()) {
                shooterEnabled = !shooterEnabled;
            }

            // Toggle auto RPM (from Limelight distance)
            if (gamepad2.yWasPressed()) {
                autoRpm = !autoRpm;
            }

            // Manual RPM: +/- 50 per press, and turns auto RPM off
            if (gamepad2.dpadUpWasPressed()) {
                targetRPM += 50;
                autoRpm = false;
            }

            if (gamepad2.dpadDownWasPressed()) {
                targetRPM -= 50;
                autoRpm = false;
            }

            // Auto RPM keeps the last value if the tag is lost
            if (autoRpm && goal.visible && goal.distanceIn > 0) {
                targetRPM = rpmForDistance(goal.distanceIn);
            }

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

                double feedforward = (targetRPM * kV) + kS;
                double proportional = rpmError * kP;
                double desired12VPower = feedforward + proportional;

                double currentVoltage = batteryVoltageSensor.getVoltage();
                double voltageScale =
                        (currentVoltage > 8.0) ? (12.0 / currentVoltage) : 1.0;

                shooterPower = Range.clip(
                        desired12VPower * voltageScale, 0.0, 1.0);
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

            // Reverse spinner
            if (gamepad2.rightBumperWasPressed()) {
                spinner.setPower(-1.0);
            } else if (gamepad2.rightBumperWasReleased()) {
                spinner.setPower(0.0);
            }

            // Unjam: reverse shooter, spinner, and intake briefly, then stop all
            if (gamepad2.backWasPressed()) {
                spinner.setPower(-1.0);
                intake.setPower(-1.0);
                shooter.setPower(-1.0);

                sleep(200);

                stopAllMotors();
                shooterEnabled = false;
            }

            // =====================================================
            // RPM-GATED SHOOTER FEEDER (hold GAMEPAD 2 B)
            // =====================================================

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
            // TELEMETRY
            // =====================================================

            telemetry.addLine("===== LIMELIGHT =====");
            telemetry.addData("Target", targetName());
            telemetry.addData("CELL tags seen", goal.tagCount);
            if (goal.visible) {
                telemetry.addData("Goal", aimed ? "AIMED" : "SEEN, NOT AIMED");
                telemetry.addData("tx (deg off center)", "%.1f", goal.tx);
                telemetry.addData("ty (deg up)", "%.1f", goal.ty);
                telemetry.addData("Distance", "%.0f in", goal.distanceIn);
            } else {
                telemetry.addData("Goal", "NOT SEEN");
            }
            telemetry.addData("Auto-aim (GP1 LT)", aiming ? "ON" : "off");
            telemetry.addData("Auto RPM (GP2 Y)", autoRpm ? "ON" : "off");

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData(
                    "Shooter Status",
                    shooterEnabled
                            ? (atTargetSpeed ? "READY TO FIRE!" : "SPINNING UP / RECOVERING")
                            : "OFF"
            );
            telemetry.addData("Target RPM", "%.0f RPM", targetRPM);
            telemetry.addData("Current RPM", "%.0f RPM", currentRPM);
            telemetry.addData("Shooter Power", "%.3f", shooterPower);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltageSensor.getVoltage());
            telemetry.addData(
                    "Feeder",
                    gamepad2.b
                            ? (shooterEnabled && atTargetSpeed ? "FEEDING" : "WAITING FOR RPM")
                            : "OFF"
            );

            telemetry.update();
        }

        limelight.stop();
    }

    // =====================================================
    // LIMELIGHT HELPERS
    // =====================================================

    /** What the Limelight sees of our alliance's goal tag this loop. */
    private static class GoalTarget {
        boolean visible = false;
        int tagCount = 0;       // how many of the CELL's 4 tags are visible
        double tx = 0;          // degrees left(-) / right(+) of center (average of visible tags)
        double ty = 0;          // degrees below(-) / above(+) center (average of visible tags)
        double distanceIn = 0;  // estimated floor distance to the CELL
    }

    /** Picks the 4 tag IDs of the CELL chosen in INIT. */
    private void selectTargetCell() {
        if (blueAlliance) {
            targetTags = farCell ? BLUE_FAR_CELL_TAGS : BLUE_AUDIENCE_CELL_TAGS;
        } else {
            targetTags = farCell ? RED_FAR_CELL_TAGS : RED_AUDIENCE_CELL_TAGS;
        }
    }

    private String targetName() {
        return (blueAlliance ? "BLUE " : "RED ")
                + (farCell ? "far CELL" : "audience CELL")
                + " (tags " + targetTags[0] + "-" + targetTags[targetTags.length - 1] + ")";
    }

    private boolean isTargetTag(int id) {
        for (int t : targetTags) {
            if (t == id) return true;
        }
        return false;
    }

    /**
     * Looks for the chosen CELL's AprilTags and ignores every other tag.
     * If several of the CELL's tags are visible, aims at their average
     * position, which is roughly the middle of the CELL.
     */
    private GoalTarget findGoal() {

        GoalTarget goal = new GoalTarget();

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return goal;
        }

        double txSum = 0;
        double tySum = 0;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (isTargetTag(tag.getFiducialId())) {
                txSum += tag.getTargetXDegrees();
                tySum += tag.getTargetYDegrees();
                goal.tagCount++;
            }
        }

        if (goal.tagCount > 0) {
            goal.visible = true;
            goal.tx = txSum / goal.tagCount;
            goal.ty = tySum / goal.tagCount;
            goal.distanceIn = distanceToGoal(goal.ty);
        }

        return goal;
    }

    /** Distance along the floor from the camera to the goal tag, in inches. */
    private double distanceToGoal(double tyDeg) {

        double angleRad = Math.toRadians(CAMERA_ANGLE_DEG + tyDeg);
        if (angleRad <= 0.01) {
            return 0;   // tag at or below the camera's level: can't estimate
        }

        return (GOAL_TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleRad);
    }

    /** Looks up the shooter RPM for a distance, blending between table rows. */
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
                return RPM_TABLE_RPM[i] + t * (RPM_TABLE_RPM[i + 1] - RPM_TABLE_RPM[i]);
            }
        }

        return RPM_TABLE_RPM[last];
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


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoTest extends LinearOpMode {

    // ================= HARDWARE =================

    private Servo blocker;

    private static final double BLOCKER_DOWN = 0.2;
    private static final double BLOCKER_UP = 0.5;

    // ================= SHOOTER CONSTANTS =================

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_ACHIEVABLE_RPM = 5200.0;

    private double targetRPM = 2250.0;
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

        blocker = hardwareMap.get(Servo.class, "blocker");

        // ================= MOTOR DIRECTIONS =================

        telemetry.addLine("Servo TeleOp Ready");
        telemetry.update();

        waitForStart();

        // ================= MAIN LOOP =================

        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()){
                blocker.setPosition(BLOCKER_UP);
            }
            if(gamepad1.bWasPressed()){
                blocker.setPosition(BLOCKER_DOWN);
            }
            telemetry.update();
        }
    }


}

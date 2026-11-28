
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Motor Test", group = "TeleOp")
public class MotorTest extends LinearOpMode {

    // ================= HARDWARE =================

    private DcMotor motor;

    @Override
    public void runOpMode() {

        // ================= HARDWARE INITIALIZATION =================

        motor = hardwareMap.get(DcMotor.class, "backleft");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Single Motor Test Ready");
        telemetry.addLine("A = Forward");
        telemetry.addLine("B = Reverse");
        telemetry.addLine("X = Stop");
        telemetry.addLine("Y = Half Power");
        telemetry.update();

        waitForStart();

        // ================= MAIN LOOP =================

        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                motor.setPower(1.0);
            }

            if (gamepad1.bWasPressed()) {
                motor.setPower(-1.0);
            }

            if (gamepad1.xWasPressed()) {
                motor.setPower(0);
            }

            if (gamepad1.yWasPressed()) {
                motor.setPower(0.5);
            }

            telemetry.addLine("===== MOTOR TEST =====");

            telemetry.addData("Motor Power", "%.2f",
                    motor.getPower());

            telemetry.addData("Motor Direction",
                    motor.getDirection());

            telemetry.addData("Motor Position",
                    motor.getCurrentPosition());

            telemetry.update();
        }

        // ================= STOP MOTOR =================

        motor.setPower(0);
    }
}

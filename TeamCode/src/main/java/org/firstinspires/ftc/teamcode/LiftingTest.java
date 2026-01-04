package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LiftingTest")
public class LiftingTest extends LinearOpMode {

    DcMotor lifting;

    static final int MOVE_COUNTS = 200;
    int targetPosition;

    @Override
    public void runOpMode() {

        lifting = hardwareMap.get(DcMotor.class, "lifting");

        lifting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder
        lifting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set initial target BEFORE switching modes
        targetPosition = 0;
        lifting.setTargetPosition(targetPosition);

        // Now switch to RUN_TO_POSITION
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifting.setPower(0.6);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                targetPosition += MOVE_COUNTS;
                lifting.setTargetPosition(targetPosition);
                sleep(200);
            }

            if (gamepad1.dpad_down) {
                targetPosition -= MOVE_COUNTS;
                lifting.setTargetPosition(targetPosition);
                sleep(200);
            }

            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", lifting.getCurrentPosition());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SimpleServoStep", group="test")
public class PIDEncode extends OpMode {

    private Axx axon;
    private ElapsedTime pauseTimer = new ElapsedTime();

    private double targetRotation = -60;   // first movement
    private boolean moving = true;

    @Override
    public void init() {
        CRServo servo = hardwareMap.get(CRServo.class, "spinner");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "spinEncoder");

        axon = new Axx(servo, encoder);

        axon.setMaxPower(0.2);
        axon.setRtp(false);     // DISABLE PID completely
        pauseTimer.reset();
    }

    @Override
    public void loop() {

        // YOU MUST call this for encoder tracking:
        axon.update();

        double currentRotation = axon.getTotalRotation();

        if (moving) {
            // Move until target reached
            if (currentRotation < targetRotation) {
                axon.setPower(-0.20);
            } else {
                axon.setPower(0);
                moving = false;
                pauseTimer.reset();
            }

        } else {
            // Wait 1 second
            if (pauseTimer.seconds() >= 1.0) {
                targetRotation += 60; // next step
                moving = true;
            }
        }

        telemetry.addData("Total Rotation", currentRotation);
        telemetry.addData("Target", targetRotation);
        telemetry.addData("Power", axon.getPower());
        telemetry.update();
    }
}

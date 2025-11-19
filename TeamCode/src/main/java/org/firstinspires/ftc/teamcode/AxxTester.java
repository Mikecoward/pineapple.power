package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Axx ULTIMATE Clean Tester", group = "Test")
public class AxxTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- 1. SETUP ---
        telemetry.addLine("Hardware Setup...");
        telemetry.update();

        // This is the standard, correct way to get hardware.
        // Double-check that your robot configuration has a CRServo named "spinner"
        // and an AnalogInput named "spinEncoder".
        CRServo servo = hardwareMap.get(CRServo.class, "spinner");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "spinEncoder");

        Axx servoController = new Axx(servo, encoder);

        // --- 2. CONFIGURE THE PID CONTROLLER ---
        // These settings are the most important part.

        // kP is the main "engine" of the controller.
        // We are starting with a value that is large enough to overcome friction.
        // If the servo moves AWAY from the target, make this number negative (e.g., -0.015).
        double kP = 0.015;

        // kI helps fix small errors if the servo stops just short of the target.
        // We will keep it at 0 for this initial test.
        double kI = 0.0;

        // kD helps prevent the servo from overshooting. It acts like a brake.
        double kD = 0.001;

        servoController.setPidCoeffs(kP, kI, kD);

        // Set a healthy maximum power. 70% is a good, strong value for testing.
        servoController.setMaxPower(0.7);

        // Make sure Run-To-Position (RTP) mode is enabled.
        servoController.setRtp(true);

        // Timer to control the automatic stepping
        ElapsedTime stepTimer = new ElapsedTime();
        double targetAngle = 0;

        telemetry.addLine("\nSetup Complete.");
        telemetry.addLine("This test will automatically step in 60-degree increments.");
        telemetry.update();

        waitForStart();

        // --- 3. MAIN LOOP ---
        stepTimer.reset();
        servoController.setTargetRotation(targetAngle); // Set the initial target (0 degrees)

        while (opModeIsActive()) {

            // --- Automatic Stepping Logic ---
            // Every 4 seconds...
            if (stepTimer.seconds() >= 4.0) {
                // ...calculate the next target angle.
                targetAngle += 60;

                // Tell the Axx controller the new target
                servoController.setTargetRotation(targetAngle);

                // Reset the timer for the next 4-second interval
                stepTimer.reset();
            }

            // --- The Most Important Line ---
            // This single line does all the PID work. It MUST be called every loop cycle.
            servoController.update();

            // --- Telemetry ---
            telemetry.addLine(servoController.log()); // Shows Target, Current, and Power
            telemetry.addData("Time until next step", "%.1f sec", 4.0 - stepTimer.seconds());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeTest extends LinearOpMode {
    private DcMotor motorTest;

    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        int intReverse = 0;
        int intStop = 0;

        while (opModeIsActive()) {
            if (gamepad1.right_trigger == 1 && gamepad1.a) {
                motorTest.setPower(-1);
            }

            else if (gamepad1.a) {
                motorTest.setPower(1);
            }

            motorTest.setPower(0);

           /* if (gamepad1.a) {
                motorTest.setPower(0);
                intStop += 1;
                sleep(800);
                System.out.println("intStop : " + intStop);
            }

            else if(intStop % 2 == 0) {
                if (gamepad1.b) {
                    motorTest.setPower(-1);
                    intReverse += 1;
                    sleep(800);
                    System.out.println("intReverse: " + intReverse);
                }

                else if (intReverse % 2 == 0) {
                    motorTest.setPower(1);
                }
            }
            */
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}


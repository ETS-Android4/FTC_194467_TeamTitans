package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.Timer;
import java.util.TimerTask;
import java.util.List;

@Autonomous
public class AutoCamera extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Arm, Intake, Carousel;
    private Timer time = new Timer();
    private VuforiaCurrentGame vuforiaFreightFrenzy;
    static private TfodCurrentGame tfodFreightFrenzy;
    static private List<Recognition> recs;
    static int ticks = 0, level = -1;

    class Event extends TimerTask {
        public void run() {
            recs = tfodFreightFrenzy.getRecognitions();
            if (recs.size()==0 && level == -1) {
                ticks++;
            } else {
                if (ticks>=0 && ticks <= 40) level = 0;
                else if (ticks > 40 && ticks <=80) level = 1;
                else if (ticks > 80 && ticks <= 120) level = 2;
            }
            time.cancel();
            telemetry.addData("List:", recs);
            telemetry.addData("ticks:",ticks);
            telemetry.update();
        }

    }

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        RF = hardwareMap.get(DcMotor.class, "motorFrontRight");
        LB = hardwareMap.get(DcMotor.class, "motorBackLeft");
        RB = hardwareMap.get(DcMotor.class, "motorBackRight");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        Intake = hardwareMap.get(DcMotor.class, "poggy");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // camera initialization
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();
        vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // Change this to the camera name, default "Webcam 1"
                "", // webcamCalibrationFilename
                true, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.
        tfodFreightFrenzy.setZoom(1, 16 / 9);

        waitForStart();

        time.scheduleAtFixedRate(new Event(), 1, 100);

        // raise arm to prevent it from dragging across floor
        Arm.setPower(-0.25);
        // turn left
        move(-1, 1, -1, 1, 100);
        move(0,0,0,0, 100);
        // backward
        move(-1,-1,-1,-1,15);
        move(0,0,0,0,100);
        Carousel.setPower(-0.65/2);
        sleep(5000);

        // turn left
        move(-1,1,-1,1,200);
        // forward
        move(1,1,1,1,1675);
        move(0,0,0, 0,100);
        // raise arm for intake

        if (level==0||level==-1) {
            Arm.setPower(-0.25);
        } else if (level==1) {
            Arm.setPower(-0.5);
        } else if (level == 2) {
            Arm.setPower(-0.7);
        }
        // push freight out
        Intake.setPower(1);
        sleep(2000);
        // stop intake
        Intake.setPower(0);
        Arm.setPower(-0.25);

        // backward
        move(-1,-1,-1,-1, 350);
        move(0,0,0,0,100);
        // turn left
        move(-1,1,-1,1, 1300);
        move(0,0,0,0, 100);
        // backward
        move(-1,-1,-1,-1, 6500);


    }

    public void move(double LF, double RF, double LB, double RB, int sleepMS) {
        this.LF.setPower(LF);
        this.LB.setPower(LB);
        this.RF.setPower(RF);
        this.RB.setPower(RB);
        sleep(sleepMS);
    }

}
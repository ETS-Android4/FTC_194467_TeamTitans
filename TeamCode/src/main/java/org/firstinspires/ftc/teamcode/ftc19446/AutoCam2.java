package org.firstinspires.ftc.teamcode.ftc19446;

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
public class AutoCam2 extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Arm, Intake, Carousel, Turret;
    //private Timer time = new Timer();
    private VuforiaCurrentGame vuforiaFreightFrenzy;
    private TfodCurrentGame tfodFreightFrenzy;
    private List<Recognition> recs;
    //static int ticks = 0, level = -1;
    /*
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
     */

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        RF = hardwareMap.get(DcMotor.class, "motorFrontRight");
        LB = hardwareMap.get(DcMotor.class, "motorBackLeft");
        RB = hardwareMap.get(DcMotor.class, "motorBackRight");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        Intake = hardwareMap.get(DcMotor.class, "poggy");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        Turret = hardwareMap.get(DcMotor.class, "Turret");

        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);

        // camera initialization
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();
        // testing initialzation function that takes direction over a camera name + calibration file
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
        // Set min confidence threshold to 0.3
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.3, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.

        waitForStart();

        Arm.setPower(-0.5);
        sleep(500);
        Arm.setPower(0);

        recs = tfodFreightFrenzy.getRecognitions();

        // shipping hub level
        int level = -1;
        int leave = 0;
        while (recs.size()==0) {
            recs = tfodFreightFrenzy.getRecognitions();
            telemetry.addData("detections: ", recs);
            if (recs.size()>0) {
                boolean n600 = false;
                for (int i=0;i<recs.size();i++) {
                    int right = (int)recs.get(i).getRight();
                    String lbl = recs.get(i).getLabel();
                    if (right>0&&right<300) {
                        n600 = true;
                        level = 1;
                    } else if (right>300&&right<600) {
                        n600 = true;
                        level = 2;
                    } else if (right>600&&recs.size()==1) {
                        recs.clear();
                        leave++;
                    }
                }
                if (leave>100) break;
                if (!n600) {
                    level = 3;
                }
            }else{
                telemetry.addData("NO DETECTIONS :P","");
            }
            telemetry.update();
        }

        //time.scheduleAtFixedRate(new Event(), 1, 100);

        Arm.setPower(0);

        // backward
        move(-1,-1,-1,-1,430);
        move(0,0,0,0,100);
        // spin carousel
        Carousel.setPower(-0.8/2);
        sleep(4000);
        Carousel.setPower(0);

        // left strafe
        move(-1, 1, 1, -1, 300);
        move(0, 0, 0, 0, 100);
        // forward
        move(1,1,1,1,2000);
        move(0,0,0, 0,100);

        // strafe correction (left turn)
        move(-1, 1, -1, 1, 15);
        move(0, 0, 0, 0, 100);

        // raise arm for intake
        // arm level to load freight
        if (level==1) {
            move(-1, 1, 1, -1, 800);
            move(0, 0, 0, 0, 100);
            Arm.setPower(-0.5);
            sleep(750);
        } else if (level==2) {
            // strafe to shipping hub
            move(-1, 1, 1, -1, 700);
            move(0, 0, 0, 0, 100);
            Arm.setPower(-0.5);
            sleep(1800);
        } else if (level == 3||level==-1) {
            // strafe to shipping hub
            move(-1, 1, 1, -1, 750);
            move(0, 0, 0, 0, 100);
            Arm.setPower(-0.5);
            sleep(2500);
        }

        telemetry.addData("level", level);
        telemetry.update();

        // push freight out
        Intake.setPower(1);
        Arm.setPower(0);
        sleep(1000);
        // stop intake & drop arm
        Intake.setPower(0);
        Arm.setPower(0.25);
        sleep(500);

        // strafe into wall and park in shipping hub (and move arm forwards)
        Arm.setPower(-0.5);
        sleep(600);
        Turret.setPower(0.5);
        move(1, -1, -1, 1, 1200);
        move(0, 0, 0, 0, 100);
        Turret.setPower(0.2);
        Arm.setPower(0.25);
        move(1, 1, 1, 1, 2500);
        move(0, 0, 0, 0, 100);
        Arm.setPower(0);
        Turret.setPower(0);
    }

    public void move(double LF, double RF, double LB, double RB, int sleepMS) {
        this.LF.setPower(LF);
        this.LB.setPower(LB);
        this.RF.setPower(RF);
        this.RB.setPower(RB);
        sleep(sleepMS);
    }

}

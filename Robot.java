import com.ridgesoft.intellibrain.IntelliBrain;
import com.ridgesoft.intellibrain.IntelliBrainDigitalIO;
import com.ridgesoft.io.Display;
import com.ridgesoft.robotics.AnalogInput;
import com.ridgesoft.robotics.Motor;
import com.ridgesoft.robotics.RangeFinder;
import com.ridgesoft.robotics.sensors.ParallaxPing;

public class RB {

    // *********************************** Constants *************************************
    // ***********************************************************************************
    // Possible states of the finite state machine
    private static final int WAIT = 0;
    private static final int NAV_RIGHT = 1;
    private static final int CENTER = 2;
    private static final int PUT_OUT = 3;
    private static final int RETURN_BACK = 4;

    // Possible floor tags
    private static final int LINE_TAG = 0;
    private static final int CIRCLE_TAG = 1;
    private static final int NO_TAG = 2;

    // Algorithm Parameters
    private static final int GO_VELO = 10;
    private static final int TURN_VELO = 6;
    private static final int DIST_TO_SIDE_WALL = 19;
    private static final int DIST_TO_FRONT_WALL = 19;
    private static final int DIST_TO_LEFT_WALL = 30;
    private static final int DELTA_LIMIT = 5;
    private static final float GAIN = 0.9f;
    private static final float TURN_FACTOR = 6.2f;
    private static final int LINE_LIMIT = 52;

    // Bumpers
    private static final int LEFT_BUMPER = 0;
    private static final int RIGHT_BUMPER = 1;

    // Flame Limits
    private static final int flamePresent = 600;
    // ***********************************************************************************

    
    // ************************ Hardware Objects and Variables ***************************
    // ***********************************************************************************
    private static int state;

    private static IntelliBrainDigitalIO startButton;
    private static IntelliBrainDigitalIO stopButton;

    private static IntelliBrainDigitalIO lBumper;
    private static IntelliBrainDigitalIO rBumper;

    private static RangeFinder lSonar;
    private static RangeFinder fSonar;
    private static RangeFinder rSonar;
    private static int lDist;
    private static int fDist;
    private static int rDist;

    private static Motor lMotor;
    private static Motor rMotor;

    private static Display lcd;

    private static AnalogInput lineSensor;
    private static int totalLines;
    private static boolean isInRoom = false;

    private static FlameSensor flameSensor;
    private static IntelliBrainDigitalIO flameLED;

    private static IntelliBrainDigitalIO soundSensor;

    private static Motor fan;

    private static boolean isInsideRoom = false;

    private static AnalogInput uvTron;
    // ***********************************************************************************

    public static void main(String[] args) {

        // ********************** Construction of the hardware objects ***********************
        // ***********************************************************************************
        startButton = IntelliBrain.getDigitalIO(12);
        startButton.setPullUp(true);
        stopButton = IntelliBrain.getDigitalIO(11);
        stopButton.setPullUp(true);

        lBumper = IntelliBrain.getDigitalIO(1);
        lBumper.setPullUp(true);
        rBumper = IntelliBrain.getDigitalIO(10);
        rBumper.setPullUp(true);

        soundSensor = IntelliBrain.getDigitalIO(9);
        soundSensor.setPullUp(true);

        lSonar = new ParallaxPing(IntelliBrain.getDigitalIO(3));
        fSonar = new ParallaxPing(IntelliBrain.getDigitalIO(4));
        rSonar = new ParallaxPing(IntelliBrain.getDigitalIO(5));

        lMotor = new ContinuousRotationServo(IntelliBrain.getServo(1), false, 14);
        rMotor = new ContinuousRotationServo(IntelliBrain.getServo(2), true, 14);

        lcd = IntelliBrain.getLcdDisplay();

        lineSensor = IntelliBrain.getAnalogInput(7);
        totalLines = 0;

        flameSensor = new FlameSensor(flamePresent);
        flameLED = IntelliBrain.getDigitalIO(13);

        fan = IntelliBrain.getMotor(2);

        uvTron = IntelliBrain.getAnalogInput(6);
        // ***********************************************************************************


        // ********************************* State Machine ***********************************
        // ***********************************************************************************
        state = WAIT;

        while (true) {
            if (!stopButton.isSet()) {
                stop();
                state = WAIT;
            }
            switch (state) {
                case WAIT:
                    state = waitState();
                    break;
                case NAV_RIGHT:
                    state = navRightState();
                    break;
                case CENTER:
                    state = centerState();
                    break;
                case PUT_OUT:
                    state = putOutState();
                    break;
                case RETURN_BACK:
                    state = returnBackState();
                    break;
            }

            displayState();
        }
        // ***********************************************************************************
    }


    // ******************************** State Functions **********************************
    // ***********************************************************************************
    private static int waitState() {
        
        totalLines = 0;
        isInRoom = false;

        while (startButton.isSet() && soundSensor.isSet()) { 
            lDist = (int) getDist(lSonar);
            //lcd.print(1, "Dir: " + flameSensor.scan() + " Val: " + flameSensor.flameVal);
            lcd.print(1, "Val: " + lineSensor.sample());
        }

        if (lDist < DIST_TO_LEFT_WALL) rotateAng(-90);

        // Move straight a bit
        move(GO_VELO, 0);
        wait(1000);

        return NAV_RIGHT;
    }

    private static int navRightState() {

    	rDist = (int) getDist(rSonar);
        fDist = (int) getDist(fSonar);

        if (fDist < DIST_TO_FRONT_WALL)
            rotateAng(90);
        else if (!lBumper.isSet())
            bumperManeuver(LEFT_BUMPER);
        else if (!rBumper.isSet())
            bumperManeuver(RIGHT_BUMPER);

        int error = rDist - DIST_TO_SIDE_WALL;
        int delta = (int) (error * GAIN);

        if (delta > DELTA_LIMIT)
            delta = DELTA_LIMIT;
        else if (delta < -DELTA_LIMIT)
            delta = -DELTA_LIMIT;

        move(GO_VELO, delta);

        // Conditions
        int tag = getFloorTag();
        if(tag == LINE_TAG) {
            totalLines++;
            isInRoom = !isInRoom;
        }
        else if (tag == CIRCLE_TAG) {
            rotateAng(180);
            move(GO_VELO, 0);
            wait(1500);
            isInsideRoom = true;
        }

        if (isInRoom) {
            if(uvTron.sample() > 0) {
                flameLED.set();
                return CENTER;
            }
            
            rotateAng(180);
            move(GO_VELO, 0);
            wait(700);
            totalLines++;
            isInRoom = !isInRoom;
        }

        // DEBUG
        lcd.print(1, "Lines: " + totalLines);
        
        return NAV_RIGHT;
    }

    private static void followRightWallInsideRoom() {  
        rDist = (int) getDist(rSonar);
        fDist = (int) getDist(fSonar);
        lDist = (int) getDist(lSonar);
        
        int error = rDist - DIST_TO_SIDE_WALL;
        int delta = (int) (error * GAIN);

        if (delta > DELTA_LIMIT)
            delta = DELTA_LIMIT;
        else if (delta < -DELTA_LIMIT)
            delta = -DELTA_LIMIT;

        if (fDist < DIST_TO_FRONT_WALL)
            rotateAng(90);
        else if (lDist < DIST_TO_SIDE_WALL)
        	move(GO_VELO, -delta);
        else if (!lBumper.isSet())
            bumperManeuver(LEFT_BUMPER);
        else if (!rBumper.isSet())
            bumperManeuver(RIGHT_BUMPER);

        move(GO_VELO, delta);
        
        if (getFloorTag() == LINE_TAG) {
        	stop();
            rotateAng(180);
            move(GO_VELO, 0);
            wait(700);
        }  
    }

    private static int centerState() {
        int rotateValue = 15;
        switch (flameSensor.scan()) {
            case 1:
            	rotateAng(-rotateValue);
                break;
            case 2:
                rotateAng(-rotateValue);
                break;
            case 3:
                fDist = (int) getDist(fSonar);
                move(8, 0);
                if (fDist < DIST_TO_FRONT_WALL + 4) {
                    stop();
                    return PUT_OUT;
                }
                break;
            case 4:
            	rotateAng(rotateValue);
                break;
            case 5:
            	rotateAng(rotateValue);
                break;
            case -1:
            	followRightWallInsideRoom();  
                break;
        }
        // DEBUG
        lcd.print(1, "Lines: " + totalLines);
        return CENTER;
    }

    private static int putOutState() {
        fan.setPower(16);
        wait(3000);
        fan.setPower(0);
        flameLED.clear();
        
        wait(100);

        if(uvTron.sample() > 0) {
            flameLED.set();
            return CENTER;
        }

        // Direkt sol duvarı takip etmeye başla bunu dene!
	
        move(-GO_VELO, 0);
        wait(600);
        rotateAng(-90);
        
        while ((int)getDist(fSonar) > DIST_TO_FRONT_WALL + 5) {
            if ((int)getDist(lSonar) < DIST_TO_SIDE_WALL) {
                break;
            }
            move(GO_VELO, 0);
        }

        // rotateAng(-90);
        return RETURN_BACK;
    }

    private static int returnBackState() {

        lDist = (int) getDist(lSonar);
        fDist = (int) getDist(fSonar);

        if (fDist < DIST_TO_FRONT_WALL)
            rotateAng(-90);
        else if (!lBumper.isSet())
            bumperManeuver(LEFT_BUMPER);
        else if (!rBumper.isSet())
            bumperManeuver(RIGHT_BUMPER);

        int error = lDist - DIST_TO_SIDE_WALL;
        int delta = (int) (error * GAIN);

        if (delta > DELTA_LIMIT)
            delta = DELTA_LIMIT;
        else if (delta < -DELTA_LIMIT)
            delta = -DELTA_LIMIT;

        move(GO_VELO, -delta);

        // Conditions
        int tag = getFloorTag();
        if(tag == LINE_TAG) {
            isInRoom = !isInRoom;
            totalLines--;

            if (isInRoom) {
                stop();
                move(-GO_VELO, 0);
                wait(500);
                rotateAng(-180);
                isInRoom = !isInRoom;
                totalLines--;
            }

            if (isInsideRoom) {
                // Turn right a bit
                while(getDist(fSonar) > DIST_TO_FRONT_WALL)
                    move(GO_VELO, 0);
                isInsideRoom = !isInsideRoom;
            }
        }
        else if (tag == CIRCLE_TAG) {
            stop();
            return WAIT;
        }

        // DEBUG
        lcd.print(1, "Lines: " + totalLines);

        return RETURN_BACK;
    }
    // ***********************************************************************************


    // ******************************** Motor Functions **********************************
    // ***********************************************************************************
    private static void move(int power, int delta) {
        lMotor.setPower(power + delta);
        rMotor.setPower(power - delta);
    }

    private static void rotateAng(int ang) {
        if (ang < 0) {
            ang = -ang;
            rotate(-TURN_VELO);
        } else
            rotate(TURN_VELO);
        wait((int) (ang * TURN_FACTOR));
        stop();
    }

    private static void rotate(int velo) {
        lMotor.setPower(-velo);
        rMotor.setPower(velo);
    }

    private static void bumperManeuver(int b) {
        stop();
        if (b == LEFT_BUMPER)
            move(-GO_VELO, 4);
        else
            move(-GO_VELO, -4);
        wait(1000);
        stop();
    }

    private static void stop() {
        lMotor.stop();
        rMotor.stop();
    }
    // ***********************************************************************************


    // ******************************** Sensor Functions *********************************
    // ***********************************************************************************
    private static float getDist(RangeFinder s) {
        s.ping();
        wait(15);
        float dist = s.getDistanceCm();
        if (dist == -1) dist = 100;
        return dist;
    }

    private static int getFloorTag() {
        if (lineSensor.sample() < LINE_LIMIT) {
            move(GO_VELO, 0);
            wait(420);
            stop();
            if (lineSensor.sample() < LINE_LIMIT)
                return CIRCLE_TAG;
            else
                return LINE_TAG;
        }
        return NO_TAG;
    }
    // ***********************************************************************************


    // ***************************** Auxiliary Functions *********************************
    // ***********************************************************************************
    private static void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static void displayState() {
        switch (state) {
            case WAIT:
                lcd.print(0, "WAIT");
                break;
            case NAV_RIGHT:
                lcd.print(0, "NAV_RIGHT");
                break;
            case CENTER:
                lcd.print(0, "CENTER");
                break;
            case PUT_OUT:
                lcd.print(0, "PUT_OUT");
                break;
            case RETURN_BACK:
                lcd.print(0, "RETURN_BACK");
                break;
        }
    }
    // ***********************************************************************************
}

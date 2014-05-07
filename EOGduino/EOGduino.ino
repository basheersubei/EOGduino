
/**
* @author: Basheer Subei and Mengqi Xing
*
* Calibrate button starts calibration procedure (monitor the serial port for instructions)
* Arm button will start recording and detect blinks or look-up or look-down events. It does so by running a fixed-size moving window (using QueueLists) on the ADC data
* and it registers a blink if it detects a fast negative change (narrow valley), registers a look-up event if it detects a slow positive change (broad spike), and registers
* a look-down event if it detects a slow negative change (broad valley).
*
*
* Some snippets were taken from the official Arduino examples.
*/
#include <QueueList.h>


/* constants and global variables */

// variables for EOG limits and values
//float up_threshold;  // threshold for triggering an up eye event
//float down_threshold; // threshold for triggering a down eye event
//float baseline_mean; // the baseline for EOG value (after calibration)

QueueList<int> narrow_window; // holds around 400ms worth of ADC data
QueueList<int> wide_window; // holds around 800ms worth of ADC data


//const float SAMPLING_RATE = 10000; // assuming Arduino samples at 10kHz
//const float WIDE_WINDOW_TIME = 0.8; // 800ms
//const float NARROW_WINDOW_TIME = 0.4; // 400ms
//const int WIDE_WINDOW_SIZE = SAMPLING_RATE * WIDE_WINDOW_TIME; // using 0.8 seconds for wide_window time
//const int NARROW_WINDOW_SIZE = SAMPLING_RATE * NARROW_WINDOW_TIME; // using 0.4 seconds for narrow_window time
float WIDE_WINDOW_TIME;
float NARROW_WINDOW_TIME;
const int WIDE_WINDOW_SIZE = 1000;
const int NARROW_WINDOW_SIZE = 200;

float up_slope_threshold;
float down_slope_threshold;
float blink_slope_threshold; // assuming this is always negative and larger than other slopes

// input/output pins
const int calibration_button_pin = 10;
const int arm_button_pin = 11;
const int EOG_input_pin = A0;

// constants
const long delay_in_calibration = 10000;
const float threshold_coefficient = 0.5;
const long debounce_delay = 500;    // the debounce time; increase if the output flickers
const int RECORDING_TIME = 10000; // records for 10 seconds every time arm button is triggered

// stuff for calibration button and arm button (initiates either sequence)
int calibration_mode = 0;
int arm_control_mode = 0;
int calibrated = 0;

int calibration_current_button_state;             // the current reading from the input pin
int calibration_last_button_state = LOW;   // the previous reading from the input pin
long calibration_last_debounce_time = 0;  // the last time the output pin was toggled

int arm_current_button_state;             // the current reading from the input pin
int arm_last_button_state = LOW;   // the previous reading from the input pin
long arm_last_debounce_time = 0;  // the last time the output pin was toggled


//do something when a blink is detected (toggle claw)
void blinkEvent()
{
  Serial.println("Blink detected!");
}
//do something when an up event is detected (move arm up)
void upEvent()
{
  Serial.println("Up event detected!");
}
//do something when a down event is detected (move arm down)
void downEvent()
{
  Serial.println("Down event detected!");
}





// checks if the arm button was pressed
// returns true if it was pressed, false otherwise
int check_arm_button()
{
   int reading = digitalRead(arm_button_pin);

  // If the switch changed, due to noise or pressing:
  if (reading != arm_last_button_state) {
    // reset the debouncing timer
    arm_last_debounce_time = millis();
  } 
  
  if ((millis() - arm_last_debounce_time) > debounce_delay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    
    // if the button state has changed:
    if (reading != arm_current_button_state) {
      arm_current_button_state = reading;

      // only toggle the LED if the new button state is HIGH
      if (arm_current_button_state==HIGH){
        return true;
      }
    }
  } 
  
  arm_last_button_state = reading;
}


// checks if the calibration button was pressed
// returns true if it was pressed, false otherwise
int check_calibration_button()
{
   int reading = digitalRead(calibration_button_pin);

  // If the switch changed, due to noise or pressing:
  if (reading != calibration_last_button_state) {
    // reset the debouncing timer
    calibration_last_debounce_time = millis();
  } 
  
  if ((millis() - calibration_last_debounce_time) > debounce_delay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    
    // if the button state has changed:
    if (reading != calibration_current_button_state) {
      calibration_current_button_state = reading;

      // only toggle the LED if the new button state is HIGH
      if (calibration_current_button_state==HIGH){
        return true;
      }
    }
  } 
  
  calibration_last_button_state = reading;
}

// starts listening to EOG signal and acts accordingly (when EOG signal crosses threshold, it moves arm up or down)
// stops automatically after RECORDING_TIME seconds
void start_recording()
{
  Serial.println("We started recording!");
  
  // TODO set up a QueueList and keep both a wide window (500ms) and a narrow window (100ms) running from the ADC readings. Every time we read a new ADC value,
  // we add it to the queue (pop oldest value if queue is full), then evaluate what event is detected. If slope of wide window is above threshold, then it's a look-up or look-down
  // event (based on sign of slope).  If slope from narrow window is above threshold but slope from wide window is below threshold, then it is a blink. Otherwise, it's not a blink.
  
  //first empty the queues, then fill them up again.
  
  //empty the queues
  while(!wide_window.isEmpty())
    wide_window.pop();
  
  while(!narrow_window.isEmpty())
    narrow_window.pop();
    
   
  int current_reading;
  //fill them up with ADC readings again
  while(wide_window.count() < WIDE_WINDOW_SIZE)
  {
    current_reading = analogRead(EOG_input_pin);
    
    if(narrow_window.count() >= NARROW_WINDOW_SIZE)
      narrow_window.pop();
    
    wide_window.push(current_reading);
    narrow_window.push(current_reading);
  }
  
  //now both windows are full
  
  //start looking for events (above slope thresholds) for RECORDING_TIME seconds, then stop 
  
    long starttime = millis();
    long endtime = starttime;

    while ((endtime - starttime) <= RECORDING_TIME) // do this loop for up to 10s
    {

      current_reading = analogRead(EOG_input_pin);

      float wide_oldest; // the oldest value in the window (soon to be taken out)
      float narrow_oldest; // the oldest value in the window (soon to be taken out)
      float wide_slope;
      float narrow_slope;
      
      //calculate wide slope
      if(wide_window.count() >= WIDE_WINDOW_SIZE)  
      {
        wide_oldest = wide_window.pop(); // save the oldest value we had in the window before throwing it away
        //slope is (y2 - y1) / (x2 - x1) or the same as (current_reading - oldest) / (WIDE_WINDOW_TIME)
        wide_slope = (current_reading - wide_oldest) / (WIDE_WINDOW_TIME);
      }
      //calculate narrow slope
      if(narrow_window.count() >= NARROW_WINDOW_SIZE)  
      {
        narrow_oldest = narrow_window.pop(); // save the oldest value we had in the window before throwing it away
        //slope is (y2 - y1) / (x2 - x1) or the same as (current_reading - oldest) / (WIDE_WINDOW_TIME)
        narrow_slope = (current_reading - narrow_oldest) / (NARROW_WINDOW_TIME);
      }

    //now that we have the wide and narrow slopes, look for events
    // for blinking event, if narrow window slope is less than (more negative) blink_slope_threshold but wide window slope is greater than (less negative) blink_slope_threshold
    // for up event, if wide window slope is greater than up_slope_threshold
    // for down event, if wide window slope is less than (more negative) down_slope_threshold

    //if blink
    if(wide_slope > blink_slope_threshold  &&   abs(narrow_slope) > abs(blink_slope_threshold))
      blinkEvent();
    //else if up event
    else if(wide_slope > up_slope_threshold)
      upEvent();
    //else if down event
    else if(wide_slope < down_slope_threshold)
      downEvent();


      endtime = millis();
    } //end while (done after RECORDING_TIME seconds)
  
  
  
  // TODO detect when threshold is crossed, then move the arm
  
  
  
  Serial.println("Recording stopped!");
}

// TODO change calibration to use slope from two windows instead of amplitude threshold. Also need to add blinking to the calibration sequence
// initiates calibration sequence. reads the baseline signal for 5 seconds, then gets the look-up maximum value, then gets the look-down minimum value.
int calibrate()
{
    Serial.println("Calibration started!");            
   
   // loop for 5 seconds and calculate baseline
    long starttime = millis();
    long endtime = starttime;
    int number_of_readings = 1;
    int current_reading;
/*  OLD PRE-SLOPE STUFF    
    while ((endtime - starttime) <= delay_in_calibration) // do this loop for up to 5s
    {

      current_reading = analogRead(EOG_input_pin);   
      //Serial.println(current_reading);    

      //recalculate mean based on new value
      baseline_mean = (baseline_mean * number_of_readings + current_reading) / ++number_of_readings;      
      endtime = millis();
    } //end while baseline_mean
    
    Serial.print("Done calculating baseline! The baseline_mean is: ");
    Serial.println(baseline_mean);    
*/
    //calibrate UP
    Serial.println("Calibrating up! Look up multiple times!");            
   
   // loop for 5 seconds and calculate threshold
    starttime = millis();
    endtime = starttime;
    float up_slope_max = 0; 
   // set up wide window, fill it up
   // then find highest slope as we go in time (for 5 seconds)
   // set a threshold slope of 80% of that max_slope
    
    //while ((endtime - starttime) <= delay_in_calibration) // do this loop for up to 5s
    for(int i=0; i < sizeof(up_array) / sizeof(int); i++)
    {
      delayMicroseconds(500);
//      current_reading = analogRead(EOG_input_pin);
      current_reading = up_array[i];
      
      float oldest; // the oldest value in the window (soon to be taken out)
      float current_slope;
      int window_filled = 0;
      if(wide_window.count() >= WIDE_WINDOW_SIZE)  
      {
        
        if(!window_filled)
        {
          WIDE_WINDOW_TIME = millis() - starttime;
          window_filled = 1;
        }
        //Serial.println(millis() - starttime);
        
        oldest = wide_window.pop(); // save the oldest value we had in the window before throwing it away

        //slope is (y2 - y1) / (x2 - x1) or the same as (current_reading - oldest) / (WIDE_WINDOW_TIME)
        current_slope = (current_reading - oldest) / (WIDE_WINDOW_TIME);
  
        up_slope_max = current_slope > up_slope_max ? current_slope : up_slope_max;
/*      line above is exact same as:
        if(current_slope > up_slope_max)
            up_slope_max = current_slope;
        else
          up_slope_max = up_slope_max;
*/
      }

      wide_window.push(current_reading);
  // OLD PRE-SLOPE STUFF  
      //save the maximum value we ever get
      // if the current_reading is greater than up_max, then set up_max to current_reading. Else, keep it as it is.
//      up_max = current_reading > up_max ? current_reading : up_max;
//      Serial.println(up_max);    
      
      endtime = millis();
    } //end while
    
      //empty the queues
    while(!wide_window.isEmpty())
      wide_window.pop();
  
    Serial.println(wide_window.count());

    up_slope_threshold = up_slope_max * threshold_coefficient;
    Serial.print("Calibrating up done! up_slope_threshold is: ");            
    Serial.println(up_slope_threshold);


    //calibrate DOWN
    Serial.println("Calibrating down! Look down multiple times!");            
   
   // loop for 5 seconds and calculate threshold
    starttime = millis();
    endtime = starttime;
//    number_of_readings = 1;
//    float up_max = baseline_mean;
    float down_slope_min = 0; 
   // set up wide window, fill it up
   // then find highest slope as we go in time (for 5 seconds)
   // set a threshold slope of 80% of that max_slope
    
    //while ((endtime - starttime) <= delay_in_calibration) // do this loop for up to 5s
    for(int i=0; i < sizeof(down_array) / sizeof(int); i++)
    {      
      delayMicroseconds(500);

      //current_reading = analogRead(EOG_input_pin);
      current_reading = down_array[i];
      float oldest; // the oldest value in the window (soon to be taken out)
      float current_slope;
      int window_filled = 0;
      if(wide_window.count() >= WIDE_WINDOW_SIZE)  
      {
        
        if(!window_filled)
        {
          WIDE_WINDOW_TIME = millis() - starttime;
          window_filled = 1;
        }
        //Serial.println(millis() - starttime);
        oldest = wide_window.pop(); // save the oldest value we had in the window before throwing it away
        
        //slope is (y2 - y1) / (x2 - x1) or the same as (current_reading - oldest) / (WIDE_WINDOW_TIME)
        current_slope = (current_reading - oldest) / (WIDE_WINDOW_TIME);
  
        down_slope_min = current_slope > down_slope_min ? current_slope : down_slope_min;
/*      line above is exact same as:
        if(current_slope > up_slope_max)
            up_slope_max = current_slope;
        else
          up_slope_max = up_slope_max;
*/
      }

      wide_window.push(current_reading);
  // OLD PRE-SLOPE STUFF  
      //save the maximum value we ever get
      // if the current_reading is greater than up_max, then set up_max to current_reading. Else, keep it as it is.
//      up_max = current_reading > up_max ? current_reading : up_max;
//      Serial.println(up_max);    
      
      endtime = millis();
    } //end while

      //empty the queues
    while(!wide_window.isEmpty())
      wide_window.pop();
    
    Serial.println(wide_window.count());


    down_slope_threshold = down_slope_min * threshold_coefficient;
    Serial.print("Calibrating down done! down_slope_threshold is: ");            
    Serial.println(down_slope_threshold);
        
    

    //calibrate BLINKING
    Serial.println("Calibrating blinking! Try blinking multiple times!");            
   
   // loop for 5 seconds and calculate threshold
    starttime = millis();
    endtime = starttime;
//    number_of_readings = 1;
//    float up_max = baseline_mean;
    float blink_slope_min = 0; 
   // set up wide window, fill it up
   // then find highest slope as we go in time (for 5 seconds)
   // set a threshold slope of 80% of that max_slope
    
    //while ((endtime - starttime) <= delay_in_calibration) // do this loop for up to 5s
    for(int i=0; i < sizeof(blink1_array) / sizeof(int); i++)
    {
      delay(1);      
      
      //current_reading = analogRead(EOG_input_pin);
      current_reading = blink1_array[i];
      float oldest; // the oldest value in the window (soon to be taken out)
      float current_slope;
      int window_filled = 1;
      if(narrow_window.count() >= NARROW_WINDOW_SIZE)  
      {
        if(!window_filled)
        {
          NARROW_WINDOW_TIME = millis() - starttime;
          window_filled = 1;
        }
        
        //Serial.println(millis() - starttime);
        oldest = narrow_window.pop(); // save the oldest value we had in the window before throwing it away

        //slope is (y2 - y1) / (x2 - x1) or the same as (current_reading - oldest) / (WIDE_WINDOW_TIME)
        current_slope = (current_reading - oldest) / (NARROW_WINDOW_TIME);
  
        blink_slope_min = current_slope > blink_slope_min ? current_slope : blink_slope_min;
      }

      narrow_window.push(current_reading);
      endtime = millis();
    } //end while
    
    //empty the queue
    while(!narrow_window.isEmpty())
      narrow_window.pop();

      Serial.println(narrow_window.count());
    
    
    //check that threshold signs are correct. If not, then return 0 (so that it reruns calibrate)
    if(blink_slope_threshold > 0 || up_slope_threshold < 0 || down_slope_threshold > 0)
    {      
      Serial.println("Threshold signs are incorrect!!! Rerunning calibrate!");
      return 0;
    }
    
    blink_slope_threshold = blink_slope_min * threshold_coefficient;
    Serial.print("Calibrating blink done! blink_slope_threshold is: ");            
    Serial.println(blink_slope_threshold);

    calibrated = 1; // indicate that device has been calibrated at least once.
    Serial.println("Leaving calibration mode!");            
    return 1;
}

// on start up: set input pin modes and start serial port transmission
void setup() {
  
  pinMode(calibration_button_pin, INPUT);
  pinMode(arm_button_pin, INPUT);
  
    // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // set the printers of the queue.
  narrow_window.setPrinter (Serial);
  wide_window.setPrinter (Serial);

  Serial.println(WIDE_WINDOW_SIZE);
  Serial.println(NARROW_WINDOW_SIZE);
  delay(1000); // wait one second just to let the button pins settle...  
}

// constantly check if either buttons are pressed. If they are, then act accordingly.
void loop() {
  
if(check_calibration_button())
{
  while(!calibrate());
  //delay(1000); //wait one second to kludge-fix random calibrate button triggers
}
else if(check_arm_button() && calibrated)
  start_recording();
                   
}// end loop()

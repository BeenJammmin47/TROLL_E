# Ben's Notebook

## 8/27/26: Pitching Project Ideas
__Initial Idea Brainstorming__
We want to build a automatic window blinds system that can detect sunlight coming through the window and close the blinds accordingly. This should account for when the sun is out and too bright, as well as when it is night time and there is no light outside. Both of these senarios should result in fully closing the blinds, with the varying light levels between them having their own blinds closed percentage.
We want to include a mobile app for this, so the user has control over it if they want the blinds at a different level.
This could also be implemented with solar power, so that it can recharge during bright days. Ideally the system still runs on battery or some other source of power, but with the location it will preside, getting solar power makes sense to us.

## 9/2/26: New Project Idea
__Changed Project Idea__
__TROLL E__
The initial project was scrapped as it was very difficult to demonstrate, being that demos would occur in the winter with less sunlight and our testing phases would depend on weather. 
We pivoted to a new idea, an autonomous following cart that has sensors to detect obstacles so it can avoid them, while following the user. This design should be able to carry some form of load, like groceries or outdoor equipment so that the user is more free to enjoy their time.
We want this design to be able to traverse a few different terrains, like grass and sand, along with potentially climbing and decending stairs. The stair option we understand will be difficult, so that remains an idea for going forward.

## 9/4/26: TROLL E Features
__Figuring Out What TROLL E Should Be Able To Do__


## 9/9/26: Project Approved
__Project was Approved__
__Follow-up Questions to Consider__
Professor approved out project, but had a few follow-up questions for us to consider:
* 

## 9/16/26: Writing up Proposal
__Documenting Components__
__Functionality__


## 


## 10/7/26: Design Review
__Taking Feedback__
During our design review, a few point were brought up that we need to do more research in and see if we need or should include them to our design. Below is a list of takeaways I wrote down during the review:
* Stall current, if TROLLE missed an obstacle and is driving into it without moving forward
* If motors differ slightly, how will we ensure they all rotate at the same speed or the speed desired for omni-movement?
* Slopes, and how to close feedback loop and slow down on negative slopes
* Odometry Pods, to track distance travels, has x and y positions; this will require additional inputs into the MCU
* E-stop to cut all power (some form of autonomy, must require this)
* Low-Voltage Current protection.
We need to focus on first developing a working PCB, but will follow up on these once the design is finished

## 10/8/26: Second Order KiCad File
__KiCad Completion & Order__
This was our team's first order for a PCB that we designed. Using the parts in our design doc, along with the prebuilts we plan on buying online, we designed our PCB to integrate all of the pins for connecting sensors. 
During the design process, Tianda informed us one several issues with our design, namely that it was too large and the gerbers showed 4 layers, which we immediately corrected.
Additionally, we want to redesign the PCB to be more optimized with its layout. Unsure who this responsibility will fall to, but we need to test sensors at the same time.

## 10/12/26: Github Repo creation, along with notebooks
__Repo & Notebook Creation__
__Uploading Previous Documents and Images__
This day, I made our collective github repository to store all our revisions of our KiCad project as well as establishing a space for our notebooks to reside. Slightly later than was suggested, but better late then never. This also included uploading all relevant documents and images that we previously used for anything we did up to this point in time. 
Both our Proposal and Design Doc have been uploaded, along with the images used in each document that we used. Each of these have their own folder to separate and organize all the files.

## 10/13/26: Prep Breadboard Demo
__Functional Motor__
Daniel and I went into the lab to test our breadboard before demo, using a 12V power supply for the ESC, and the 5V pin out the ESC for our Arduino Mega. The Mega we are using for testing how to program an MCU to determine how the motor should function, as we have a physical potentiometer for testing, but will need to test PWM for our PCB as we plan on setting 3 different speeds, a following speed, a slow speed, and a stopped speed. Each of these will be static, so we will not need a ditigal potentiometer for our board, but using a physical one for demoing is easy to visualize.

## 10/14/26: Breaboard Demo
__Complexity of Whats Complete__
__Integrate Different Sensors__
__Test Ultrasonic__
We need to change out our testing board to a dev board using our MCU, which Daniel is doing immediately. Additionally, we have been recommended using another sesnor for detection, or solidifying perfect execution of the ultrasonic sensors for forward sensing. Not only do we have not enough sensors, using more varied sensors adds to complexit yand vision.
PCB is ordered and coming within a week, we will need to fully test everything before then so that we can actually test the PCB design.
While testing the Ultrasonic, we can see that it is somewhat accurate, sitting around 1-2 cm of error, but will easily read shorter than what we expect if something is slightly in view. This is also a test without a tape measure to confirm, so we will need some method to verify another day. We also noticed that it focuses on the closest obstacle in its field of view, so this will likely remain our safety sensor for obstacle detection as it sees anything close easily, while Robin looks into options we have for path finding and if that is something we are still considering at this stage.

## 10/17/26: Load Cell Prep & Integrating Motor and Ultrasonic
__Load Cell Overview__
__Coding motors to depend on Ultrasonic__
Spent some time with Robin going over the Load Cell he was shipped, thinking about how to integrate it with our motors as we intended.
Additionally, started brainstorming and coding how to integrate the Ultrasonic sensors with the motor. Established draft of code to be tested in lab tomorrow.

## 10/18/26: Ultrasonic & Motor Integration, Load Cell & Motor Integration
__Testing Motor/Ultrasonic Functionality__
__Soldering Load Cell to Test with Motor__
Tested Ultrasonic & Motor code, revising to implement as we want. Currently, there are 2 thresholds for how fast the motor should spin, being full stop at 40 cm, slow speed at 80 cm, and normal speed higher than 80. We can consider more gradual speed change later when we have all our motors we will actually use, along with the dev board or our ordered PCB to convert our current Arduino code to STM.
Began integrating the Load Cell as well, Robin soldered the combinator and HX711 together, with one load cell to test before connecting all 4.

## 10/20/26






## 10/31/26: Working Through STM Code with Dev Board
__Convert from Arduino to STM__
Continuing to convert our existing Arduino code into STM code, trying to 

## 11/1/26: Finishing Code for Progress Demo
__Dev board__
We spent this work period coding the dev board, working through how to wire each sensor and read information from them. This was a struggle for us as we didn't have the right wire connections to use the many GPIO pins as we only had wires from previous ECE courses, none of which were female type. Plan moving forward is to get those wires for the dev board and for when we get all our part for our PCB, which can happen tomorrow. 
For the progress demo, we want to show the motors working and being able to turn both forward and backward.

## 11/2/26: Progress Demo
__Finishing Motor Demo__
We came into lab early to finish the stm code to get one motor working, as combining all 4 into a single output pin didn't seem to work reliably. This work session was when we learned that the motor's we purchased are misslabeled, where the PWM(blue) signal didn't respond to receiving PWM as we expected. When we tried an analog signal instead, which started working well with one motor, so we demoed that. Moving forward, we need to use the female type wires we bought to integrate all motors with their own pins. Once the motors are functioning as we expect them to, we can see if the existing code for ultrasonic and the load cells are working, and look to increase the number of ultrasonic sensors along with including lidar into the design.

## 11/3/26: Soldering PCB
__Checking For Grounds__
We got our Digikey parts in today, so we soldered them onto our pcb. Once that was done, we started testing if the board was correct, checking for grounds and seeing what voltages were reading when power was supplied. 
From this, we noticed that the reset switch was at 0.3V instead of 3.3V, which after checking around it and seeing nothing else wrong, forced us to look into if certain parts on the board were faulty.
Prime suspect was the C13 capacitor, which is 100nF, as it is closest as the switch functioned so we didn't think to view it as a potential cause of this. Additionally, we noticed that the voltage is as if there was a 1k resistor connected in series with the 10k R3 that is connected to this net. 

## 11/4/26: Individual Progress Check
__Completed Document__
I spent the whole day finding all that I did so far fo this project. I prioritized my progress post design review, and I left out much of the testing I did with the Arduino Mega as it doesn't really relate to our STM32 chip. 
In the Progress Check, I covered the ordering I've done for the group, along with what Daneil and Robin ordered. I also talked about how I shifted from focusing more on the sensors to integrating the code on the STM chip as I have more coding experience than my partners. From this I talked about my plans going forward and trying to get both Daniel and Robin more integrated with how I'm coding our PCB and having them participate more in the debugging process.


## 11/7/26: Confirming PCB Functionality
__Checking PCB for Shorts__


## 11/9/26: Converting STM code to PCB
__MX and IDE__


## 11/11/16: Finished Switch to PCB
__UWB Arrived__
Robin soldered all the pins onto each UWB that we got

## 11/12/26: Testing PCB Code
__Loading Code onto STM__
Connecting the STM to our PCB is quite different from our dev board as the board has a built in ST-LINK, which we did not have at this time, so we needed to use the STM32CubeProgrammer to flash our code onto our board. From this, the serial terminal needed a workaround too as the USB connection we had didn't connect that either, so now we are viewing the serial output via Mac terminal.


## 11/14/26: Testing Ultrasonic
I tested the code with the ultrasonic sensor, using the center as reference and seeing if the serial output was consistant with what I expected the sensor to display.


## 11/15/26: Testing 3 Ultrasonics, LiDAR, Motor
We were short of female wires, so we couldn't test everything in tandem, but the three ultrasonic sensors all worked at the same time, and the LiDAR


## 11/16/26: UWB Testing
Began testing with the UWB PCBs. After searching online, I decided to use the Arduino IDE to program them as they had ESP32 chips on them. Through a lot of trail and error, talking with Claude as to what I should be looking for in terms of how the AT-Commands work, I got a working prototype for establishing a single tag and a single anchor. The tag when connected to my mac via USB-C, outputted the distance between the two UWB sensors to the serial terminal.

## 11/17/26: Battery Test, Integrate Steering, TROLL E Test
We have all of the sensors working individually, and the motors are spinning based on the distance sensors, so we spent this day trying to compile it all together and test the total functionality of TROLL E. 
On the serial terminal, we can see that the ultrasonic and the LiDAR sensors are seeing obstacles and choosing the closest at the most correct. We see the UWB finding the user's anchor and shutting the motors off if both tags can't find the user. The one sensor that is not very consistant is the load cells, as they often require a user input to tare it before it will show a reasonable weight. 
We decided to move on from the load cells as we want to show motor function and steering, so we implemented that based on where the user is and if either side of TROLL E was more obstructed than the other, however our wheels struggle to get grip in the places we were testing, but when propped up the wheels spin the way that we want, so we might need to add mroe weight or somehow get more grip.




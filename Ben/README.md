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


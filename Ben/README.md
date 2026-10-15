# Ben's Notebook






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
Additionally, we want to redesign the PCB to be more optimized with its layout and try to use less VIAs 

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




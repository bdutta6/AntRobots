### September 27, 2016 
This serves as an e-introduction among all of us—Magnus and I are interested to see if his swarm work can be integrated with our ant robot work to learn if collisions among densely packed task oriented collectives of robots can be used to ensure good flow, etc. I suggest that all the students and postdocs cc’d on this email self-organize to trade info on what we (they) are doing and then in the coming weeks we can all meet to see if anything emerges...The new research direction on using ant-inspired collisions in swarm robot control/localization seems very interesting.

### October 4, 2016  
Grad Groups meeting today featured a woman from the library who overviewed the different resources available to graduate students. For research in engineering, she recommended compendex and inspec as good databases.

### October 5, 2016
I was supposed to have a Skype call with JSP today to review the robotic ant systems, but he was not able to get online. Instead, we chatted on the phone, and I took down the following notes:

* Mechanically it is sort of sorted out
  *	Two of them are not working well (Echo and Charlie?)
* When you connect the batteries, you need to separate the batteries completely off of the wires
* Issues are not commented within the code

I also spent a good amount of time today studying the codebase and learning its architecture. I think I have a good understanding of it, and I love the thorough documentation. I am not sure I understand the Watch Dog Timers (WTD), and I haven’t found a satisfactory explanation of them online. I would love an explanation of them from JSP.

### October 6, 2016  
I continued reviewing the code, and Dr. Goldman showed me how to turn on the robots. There is a black switch on the side which turns it on and off, as well as a button on the main board which can be used to reset the robot.
All of the batteries were dead, so there are two things that I need to do to get started:

1. Learn how to charge the batteries
2. Make a power cable so that the robots can run off of a power supply (3.6V).

I made a power cable to hook up to the power supply, but the power supply did not provide a high enough current to power all of the systems on the robots. I had asked Will to show me how to charge the Li-Ion batteries so I could charge them while I worked on the cable, so we were able to plug them into the battery cartridge. I also tested each of the robots with fresh set of batteries to see if they were operational, and have started a Google Sheet to track the health of each robot over time.

### October 7, 2016

Will mentioned to me yesterday that the recharging process while the ants are running is not a straightforward process, and there are several files floating around on the lab desktop with tables I think might be necessary for regulating the charging method. See “C:\Users\vlinevich3\Desktop\Charging MPR121 Program vals.xlsx”.

Continued going through the documentation left by JSP and fleshing out the Robotic Ant Status to get a better feel for the architecture and pre-existing problems. I also started up a Trello Board to track proposed changes from JSP. The Trello board is populated with the “List of things that need to be done in the future” in the Ant Robot Mk.2 Software Documentation

Calling it a day. Feeling better about navigating the code. There seems to be some extraneous stuff left over from previous iterations, but I’d prefer to just get things running for now before going back and modifying working code to basically just look pretty. Best practices can be implemented after I have proven that I can get data with what we have right now.

I was hoping to hear from JSP today, but I might just shoot him an email with all of my documented questions later tonight.

### October 10, 2016

Notes from my conversation with JSP and Bahni.
Questions for JSP:

1.	Where is the correct code?  
  *	In JSP folder like Bahni showed you.

2.	Upload process?  
  *	Straightforward

3.	Where is data stored and how do I retrieve it?  
  *	Need to communicate with the Fio board, due to the lack of available pins on the Due. Talk to Bahni about this

4.	Will mentioned to me yesterday that the recharging process while the ants are running is not a straightforward process, can you tell me about the recharging procedure and what I need to do to make sure everything is properly tuned?  
  *	Bahni knows how to do this.

5.	Debugging process? I am guessing I can get a lot from the data logging capabilities of the system  
  *		Write data to Fio

6.	I was going through the code, and I found a few lines pertaining to an LSM9DS0, which wasn’t on the inventory list, and I wasn’t able to find anything on the robots which looked like the board in its documentation. Is this code from a previous version...have I even been looking at the right code?  
  *	You have, there is just a lot of old code leftover from previous iterations

7.	Watch Dog Timer? Is this a deprecated thing now?  
  *	Still needed. Maybe a bit messy, but it gets the job done.

8.	Recommended steps from JSP:  
  *	Turning behavior needs the most work
  *	Focus on getting the IMU working
  
   *	Two modes:  
     * Magnetic field tells heading  
     * Using gyroscope to tell degrees of rotation  
       * Used to turn, but neither seems to work  
*	And get to know the hardware
* Resetting behavior is a minor issue  

####Methods

TurnHeading - mainly for turning from one direction to another (180 deg). If preferGyro is true, then it will use the gyro readings to turn. If it is false, then it will use the magnetic fields heading to determine the heading of the robot as it turns. Neither of them seem to work.
Set charging station to 4.2 volts
PID needs some work, but Vadim is a better source than JSP. 
PixyCam doesnt have a large enough field of vision. Modify the mount to make this better.
CapacitiveSensors should be reliable. The point is that we want to distinguish the wall from the other ant robots. Capacitive sensors charge and discharge the copper pads on the robot. If the time to charge changes, it can detect if it’s touching, and if it is, what type of material it is touching. When you connect it to a common ground, then the readings from the sensors will be totally different.
Antcomm lets you communicate with integers fiowriteint
repository/antorobots/FioWirelessV4
Clicking motion is a reset indication.

[Guide to .md files](https://guides.github.com/features/mastering-markdown/)  

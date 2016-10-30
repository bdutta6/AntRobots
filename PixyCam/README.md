# PixyCam Firmware-y Things

The PixyCam is the hardware for the vision system on the ant robots. There is a well documented, and unresolved issue where the communication between the Due and Pixy hang in getBlocks(). The current solution is to restart the Due, which is why you will find the WDT calls sprinkled through the .ino file of the Due code. According to Vadim, the bricking issue is more frequent in the more recent versions of the firmware, though further testing should be conducted, due to the recent firmware updates.

### Uploading Firmware

As of 10/28/2016, all of the released version of the firmware are saved in this directory. At the moment, a working robot is using version 2.0.17 with roboJSP.prm. Information and methods regarding how to upload new (or perhaps old) firmware to the Pixy can be found [here](http://cmucam.org/projects/cmucam5/wiki/Uploading_New_Firmware).

### PixyCam Parameters

A number of .prm files were discovered on the lab machine, all of which were saved to this directory. I have uploaded roboJSP.prm to a robot, which runs well. I think it is important to note that the parameters, such as "Data out port" is set to a value that according to the online documentation should not work. Perhaps some undocumented changes have been made in the PixyCam software.

Information about communicating with the Pixy can be found [here](http://cmucam.org/projects/cmucam5/wiki/Porting_Guide).

Information about the Pixy serial protocol can be found [here](http://cmucam.org/projects/cmucam5/wiki/Pixy_Serial_Protocol).

Information about the Pixy/Arduino hookup can be found [here](http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_like_an_Arduino). Note that, as of 10/27/2016, the communication line used is UART. This method does not use handshaking, and is likely the issue for the bricking issue that WDT handles.



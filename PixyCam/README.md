# PixyCam Firmware-y Things

The PixyCam is the hardware for the vision system on the ant robots. There is a well documented, and unresolved issue where the communication between the Due and Pixy hang in getBlocks(). The current solution is to restart the Due, which is why you will find the WDT calls sprinkled through the .ino file of the Due code. According to Vadim, the bricking issue is more frequent in the more recent versions of the firmware, though further testing should be conducted, due to the recent firmware updates.

### Uploading Firmware

As of 10/27/2016, all of the released version of the firmware are saved in this directory. At the moment, a working robot is using version 2.0.5 with roboJSP.prm. Information and methods regarding how to upload new (or perhaps old) firmware to the Pixy can be found [here] can](https://guides.github.com/features/mastering-markdown/).

### PixyCam Parameters

A number of .prm files were discovered on the lab machine, all of which were saved to this directory. I have uploaded roboJSP.prm to a robot, which runs well. I think it is important to note that the parameters, such as "Data out port" is set to a value that according to the online documentation should not work. Perhaps some undocumented changes have been made in the PixyCam software.



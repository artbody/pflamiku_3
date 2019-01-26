
# pflamiku 3
is a automatic watering system for indoor plants
this here is the C Implementation of the functions needed by the hardware with an STM32L031

# Its main features are:

* **FSM based user interface and workflow**: clearly and unambiguously defined behaviour.
* **Minimal Memory Requirements**:  footprint of 20 kBytes.
* **STM32L031**  a very low power microcontroller.

* Brief   
 This software is for measure 3 or 4 weight scales.
 Depended on the min and max programmed weight of the plants
 it starts a pump which is pumping water to the plant until it's
 max weight value is reached.
 The min and max values can be programmed very easy by one switch.
 Put a plant with dry plant soil on the device. Put the switch in prog position.
 The pump automatically begins to pump water to your plant.
 If the  potting soil is wet enough put the switch in position run.
 From now on the device waters your plant automatically between these two
 programmed min and max values. Each time the weight goes under the minimum
 the pump runs until the maximum or a timeout is reached.
 The timeout value is that time doubled, that the device needed
 while you programmed it. This is thought as a security feature.
 In example if the water storage tank is empty
 Another feature is that the device is going in deep sleep mode if
 there is not enough ambient light. Normally in the night.
 That switch value can be programmed as well.
 Therefore you've to wait for the according twilight condition.
 Disconnect the power plug, put the switch in PROG mode, connect
 the power plug again. Wait until the red LED blinks. Then put the switch
 in normal run mode.
 That's it.
 Now the device doesn't start until it is brighter then the twilight value.
 Another automatic feature is the brightness of the lED. It is adjusted automatically
 in respect to the ambient light.


#The Hardware is also available under the terms of CC BY SA NC 4.0 
[PCB and Schematic on EasyEDA](https://easyeda.com/artbody/pflamiku_3er_stm_2018-09-18)

TODO 
Reimplementing the FSM with 
# Web-Based Modeling Tool and Code Generator
The FW Profile Editor is available as a web-based tool to help design FW Profile state machines and procedures and to automate the generation of the C-code which configures them. The tool can be accessed from [here](http://pnp-software.com/fwprofile/editor/).

If you want to help me to bring this Produkt to market please contact the developers at this [e-mail address](mailto:artbody@gmail.com).


# Ownership
The owner of the project is [Atelier Merath](http://artbody.de/).

# License
Use of this software is granted under the terms of CC BY SA NC 4.0 see [LICENSE](LICENSE).

# Deprecation

This library has been replaced by a new project that redesigns the library. It started with the [USB-C Explorer](https://github.com/ReclaimerLabs/USB-C-Explorer/tree/master/firmware/USB-C%20Explorer). Then that firmware was ported to [Arduino](https://github.com/graycatlabs/usb-c-arduino). This new version is much easier to port to new platforms, and so it recommended for new projects. 

# Introduction

This library is a port of the [**Google Chrome EC library**](https://www.chromium.org/chromium-os/ec-development). The goal is port the code to C++ and packge it as a library that can be used with Arduino, Particle, and other embedded applications. This code is very much a work in progress. Not all of it has been tested. 

The FUSB302B is a USB Type-C port controller and BMC PHY. It allows configuring a USB-C port and sending USB Power Delivery messages. This library supports all the basic functionality. In addition, it supports the USB Type-C Port Manager (TCPM) interface, which should allow for this and other, similar libraries to be used with the more general [**USB Power Delivery library**](https://github.com/ReclaimerLabs/USB_PD). 

# Example Usage

The included example shows some basic usage of this library. Without the USB PD library, any received message doesn't make much sense. An Apple USB-C laptop charger will send unsolisicated Source_Capabilities messages. These can be decoded manually by referring to the USB Power Delivery specification.  

# Next Development Steps

The next steps are to remove the Arduino-specific references to make the code more platform agnostic. 

# Questions, Comments, and Contributions

Pull requests are welcome. If you have questions or comments, you can email me directly at jason@reclaimerlabs.com. 

# cs350
CS 350 Emerging Sys Arch &amp; Tech


Summarize the project and what problem it was solving.
  I have two projects uploaded to this repository; my flicker and thermostat projects. 
  

What did you do particularly well?
  The flicker project is somewhat simple, and only meant to be an introduction to TI's API resources, and how
  to utilize them in an embedded C with Code Composer Studio. However, the thermostat project was I felt a 
  good example of how interrupts can be utilized in a state machine.
  
  I utilzied the timer inturrput in the while loop to periodically run certain functions, like sampling the
  thermometer and adjusting the set temperature if a button was pressed. These functions are completed at 
  different rates by utilizing an iterator. 
  
Where could you improve?
  I need to utilize available API's integrate into functions instead of doing so much line by line coding.


What tools and/or resources are you adding to your support network?
  This was a great experience in learning what resources might be provided by the different microcontroller 
  manufacturers. Using the TI sysconfig resource I waas able to aquire a good amount of pregenerated code
  in Code Composer's resource options.
  

What skills from this project will be particularly transferable to other projects and/or course work?
  I am sure that I'll see more purpose driven IDE's like Code Composer that make the process much
  easier to work with.
  
How did you make this project maintainable, readable, and adaptable?
  I took extra care with the thermostat code to make it uniform, readable, and commented. Also, nearly 
  all of it's functions are in the form of functions. 

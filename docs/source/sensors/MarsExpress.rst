MEX HRSC
========
* HRSC has 9 different filters. Each has it's own instrument id, as well as
  the main/"HEAD" camera composing those filters. There is also another
  "SRC" channel, making-up a total of 11 distinct sensors. It is very
  important to understand which code is needed when/where.

* HRSC is a variable rate line scanner, and so does not maintain one exposure
  duration, but rather differing exposure durations per line. This
  information is stored within the individual records in the image data
  itself, with the the first 8 bytes making up the double presicion
  ephemeris time that the line exposure was started, and the next 4 bytes
  making up the float containing that line's exposure duration.
 
* The SRC channel is a high resolution framing sensor.

Processing HRSC Images
----------------------

Processing SRC Images
---------------------

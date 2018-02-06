compiling the codes and run: 
go to "/build"
type "cmake .."
type "make"
run "./ZED_KICK_START <port1> <port2>"
<port1> and <port2> by default are 2111 and 2112, 
port1 is for receiving client data (send on the matlab side), 
port2 is for sending data to client (receive on matlab side)

Note: You need to run this first before running the matlab simulink 
      because this is the server!

Note to Avinash: For the matlab simulink, simply replace the constant input 
                 module with the variable. This variable must be the height
                 and angle of the CAMERA's left lens. The angle is zero 
                 horizontally and positive when the camera points down.
                 Also note that the variable is a single scalar value encoded
                 like this: sign(angle) * (height * 10 + abs(angle)). My code
                 will decode this scalar value into height and angle.

CMakelists.txt - A txt file necessary to make and compile the codes
/build - needed for compiling, the executable is stored in here
/vid - generated videos after running the code are stored in here
/src/main.cpp - the main program codes

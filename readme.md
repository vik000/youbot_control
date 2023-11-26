# Control KUKA youBOT
main.py runs the whole interface. Make sure CoppeliaSIM simulation is running. This only works in PC. It should be easy to change the .dll for the equivalent in mac, I have not managed to make it work on an M2.
Controls should be quite intuitive. The only one I have not managed to make work is the grip. The control is not individual for each finger. It looks like one depends on the other, but no instruction moves the lower one. 

The interface is pyQT. That creates the interface and the connection to CoppeliaSIM, but the meat is all inside the youBOT class (inside the API dir). That one has a hiearchical structure, with a YouBOT class controling the whole thing. It is mostly a collection of joints, with the arm being a collection of its own.
Joints have a range that I googled (wikipedia), and the ranges and sizes are set according to that, which is why the joint ranges in the arm can be defined by a slider. The movement is triggered by small shots, which is easy to change if we want. 
Those are simple move and stop functions. 

# duckiebot_lab
Our experiments on the Duckiebot

This folder contains a Cmake file, that is all that is needed to compile the project.
Add this line at the bottom of the Cmake file of the chilitags directory.
add_subdirectory({path to chilitags}/myfiles)

Now create a new Makefile using ccmake, and make the entire chilitags project again - it only recompiles what is new. There may be a better way but I havent figured it out yet.

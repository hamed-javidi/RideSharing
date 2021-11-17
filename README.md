# RideSharing

To run the simulator you need to install the below requirment on linux OS.

Installation Steps:

1- Install g++
2- Install GLPK: 
        1. install from glpk-4.65.tar.gz
        2. tar -xzvf glpk-4.65.tar.gz
        3. cd glpk-4.65/ 
        4. ./configure 
        5. make
        6. make check
        7. sudo make install
        8. glpsol –version
        
3- install GLPK: (You can download the latest version 4.65 from here: http://ftp.gnu.org/gnu/glpk/ )
    1. gpg --verify glpk-4.65.tar.gz.sig
    2. tar -xzvf glpk-4.65.tar.gz
    3. cd glpk-4.65/ 
    4. ./configure 
    5. make
    6. make check
    7. sudo make install
    8. glpsol –version 

4- METIS 5.1(You can download the latest version 4.65 from here: http://glaros.dtc.umn.edu/gkhome/metis/metis/download )
    1. tar xvf metis-5.1.0.tar
    2. cd metis-5.1.0/
    3. Please modify mefis.h from this path: include/metis.h    
       find this line: #define IDXTYPEWIDTH 32    to    #define IDXTYPEWIDTH 64
    4. make configure
    5. make
    
5- Clone the the simulator project from github to your computer.

6- Open terminal and go to the RideSharing-master folder.
7- Run this command in the terminal: 
    1. make
    2. cd example
    3. make
    4.  .\launcher
    
8- You can see the BBO algorithm in this path: example/BBO

Refrence:

Please refrence the below papers if you are using this project:
1- H Javidi, D Simon, L Zhu, Y Wang, A multi-objective optimization framework for online ridesharing systems, IEEE International Conference on Big Data and Smart Computing (BigComp)

2- James J. Pan, Guoliang Li, Juntao Hu, Ridesharing: simulator, benchmark, and evaluation, Proceedings of the VLDB EndowmentVolume 12Issue 10June 2019 pp 1085–1098

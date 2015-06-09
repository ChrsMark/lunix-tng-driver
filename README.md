# lunix-tng-driver
Character device driver for OsLab NTUA course 2015 (http://www.ece.ntua.gr/el/education/undergraduate?view=ugcourse&id=73) 

In this project we implement a character device driver for linux.
The goal is to receive raw data from a system of sensors (light-batt-temp), to "cook" the data 
and then provide them to user space. 
Our implementation takes care of what would happen if more than one users make use of the device, by using locks and spinlocks appropriately.


Authors : Nikolaos Papadis (nikpapadis@gmail.com), Chris Mark (chrs.markx86@gmail.com)
(a lot of code is given from the course director <vkoukis> as a skeleton. Each file refers explicitly its author.) 

School of ECE, National Technical University of Athens


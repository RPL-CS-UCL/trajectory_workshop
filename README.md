# trajectory_workshop
Workshop on trajectory following for legged robots. This README has instructions on setting up your computer to run the simulation. It also has the questions for the lab activities. It finally has some brief notes on running your code on the real robots. Tough the teaching assistants will be the ones that facilitate with that, your code will need to have a certain structure that is elaborated on.

## Environment Setup

These are instructions to setup the software on your computer needed to complete the lab. Windows, Mac, and Unix systems should work.

1. Install Python3.12 on your computer. This version of Python is supported on Windows, Mac, and various Linux distributions. Find it [here](https://www.python.org/downloads/).
2. Install Git on your computer, link [here](https://git-scm.com/install/mac).
3. Download the code that we have written for this lab, by opening a terminal (on windows use command prompt) and changing to a known path on your computer you can access (the command `cd` changes the directory, so `cd C:\Users\michael\Documents` changes me to my documents folder in Windows as my username is Michael). "Clone" the repository, meaning download it by running this command in the terminal in the directory you've chosen: `git clone https://github.com/MZandtheRaspberryPi/trajectory_workshop.git`.
4. Create a virtual environment for this lab (a copy of python with the libraries we need) by moving into the directory you've cloned `cd trajectory_workshop` and running the command `python3 -m venv venv`. Activate the environment, on windows `.venv\Scripts\activate` and on Mac/Unix `source venv/bin/activate`. This will use the virutal environment we've created for any python invocations.
5. Install dependencies of this lab: `python3 -m pip install -r requirements.txt`.
6. (optional) Install an interactive development environment of your choice to write and run code. This is optional as you can use a text editor like notepad on Windows or the inbuilt text-editor on Mac/Unix.


## Labratory Questions

This is the lab content with guided questions on the activities and trajectories.

Run main.py. Why does the path change each time I run it if i am always passing the same comand? (Hint: look at the intiailization variable `noise_cfg`) and how it is used in the `step` function. What are some scenarios where a robot in the real world might not move exactly forward when commanded to?

## Real Robot Setup

This is what the students need to submit to the teaching assistants to run their code on the real robot.
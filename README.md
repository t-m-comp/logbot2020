# Overview
This is a repository to create Arduino codes for [machine learning enabled bio-loggers](https://www.nature.com/articles/s42003-020-01356-8). After creating the codes, you can compile them by using Arduino IDE. 

# Files
- PenbotV1BoardsTxtFile: Arduino board settings. Please add the content to your boards.txt
- logbot_*: Examples of generated ino files
- src_txt: It contains original Arduino codes. They are used to generate Arduino codes based on specified settings.
- ParseLogdata.exe: A program for converting binary files generated by a logger to CSV files. Please place this program in the root of the data folder and execute it.
- build_ino.py: A python code to generate Arduino codes based on specified settings. As for settings, please check by `python build_ino.py -h`. 
- default_args.txt: It stores default settings for code generation.
- *.cpp: Example decision tree codes.

# How to generate Arduino codes
You can simply generate codes by:

`python build_ino.py [options]`

As for options, please check by: 

`python build_ino.py -h` 



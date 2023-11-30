# Creating Decision Tree
## Building low_cost_trees package
Before creating trees, you should build a package for learning a low cost decision tree. Please go to `low_cost_trees` folder in the parent folder.
## Edit RunExperiment.py
Please edit RunExperiment.py according to your environment.

`base_dir = os.path.join("d:", "LogbotTreeBuilding")`: Please indicate the location of this folder.

`data_type = "ACC" #"GPS" #"MAG"`: Please select which sensor to activate a camera. 

`dataset =  "Example_shearwater"`: Please indicate a folder name containing your dataset. If you select "ACC" in data_type, the folder should be in "ACC_features".

`file_dict = {"normal": os.path.join(acc_dir, "normal.csv"),
                            "random": os.path.join(acc_dir, "normal.csv"),
                            "validation": os.path.join(acc_dir, "normal.csv"),
                            }`: Please specify dataset files used for train (random) and test (normal).  
                          

## Execution
Using the following command to generate decision trees.

`python RunExperiment.py`


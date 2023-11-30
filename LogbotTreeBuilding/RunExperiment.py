import os
import BuildClassifier
from multiprocessing import Process

if __name__ == '__main__':
    data_type = "ACC" #"GPS" #"MAG"
    tree_type = "default_tree" #"default_tree" #rf_tree for low cost tree, default tree for default sklearn decision tree

    dataset =  "Example_shearwater"
    upsample = False # True

    base_dir = os.path.join("d:", "LogbotTreeBuilding") 
    acc_dir = os.path.join(base_dir, "ACC_features", dataset)
    gps_dir = os.path.join(base_dir, "GPS_features", dataset)
    mag_dir = os.path.join(base_dir, "MAG_features", dataset)

    remove_features = None

    if dataset == "Human":
        target_class_string = "walk" #"foraging" #sets which class is expected to trigger the camera
        secondary_class_string = None #"flying" #used to identify a secondary class that might need to be detected prior to activating the camera for the target class

        if data_type == "ACC":
            training_data_options = [ "random" ] #"randomVariance" ]
            testing_data_options = [ "normal" ]

            if upsample:
                file_dict = {"normal": os.path.join(acc_dir, "Human_Logbot_25Hz_acc_normal_upsampledTo31Hz.csv"),
                            "random": os.path.join(acc_dir, "Human_Logbot_25Hz_acc_random_upsampledTo31Hz.csv"),
                            "validation": os.path.join(acc_dir, "Human_Logbot_31Hz_acc_normal.csv"),#"val_features.csv"),
                            }
            else:
                file_dict = {"normal": os.path.join(acc_dir, "Human_Logbot_25Hz_acc_normal.csv"),
                            "random": os.path.join(acc_dir, "Human_Logbot_25Hz_acc_random.csv"),
                            "validation": os.path.join(acc_dir, "Human_Logbot_31Hz_acc_normal.csv"),#"val_features.csv"),
                            }

            threshold_list = [1250, 1000, 900, 800, 700, 600, 500]

        elif data_type == "GPS":
            training_data_options = [ "normal" ] #"randomVariance" ]
            testing_data_options = [ "normal" ]

            file_dict = {"normal": os.path.join(gps_dir, "Human_Logbot_25Hz_gps.csv"),
                        "validation": os.path.join(gps_dir, "Human_Logbot_25Hz_gps.csv"),#"val_features.csv"),
                        }
            threshold_list = [4000, 3000, 2500, 2000, 1500, 1000]

    elif dataset == "Example_shearwater":
        target_class_string = "foraging" #sets which class is expected to trigger the camera
        secondary_class_string = "flying" #used to identify a secondary class that might need to be detected prior to activating the camera for the target class

        if data_type == "ACC":
            training_data_options = [ "random" ] #"randomVariance" ]
            testing_data_options = [ "normal" ]

            if upsample:
                pass
            else:
                file_dict = {"normal": os.path.join(acc_dir, "normal.csv"),
                            "random": os.path.join(acc_dir, "normal.csv"),
                            "validation": os.path.join(acc_dir, "normal.csv"),
                            }

            threshold_list = [1250, 1000, 900, 800, 700, 600, 500]




    number_trees_per_setting = 3 #allows you to try each possible setting multiple times
    vary_class_weights = True #allows you to try different weights for the classes, setting a higher weight for the target class
    
    #leave_one_out: run leave one id out cross validation using tree_type
    #build_final: first runs leave one id out cross validation using default trees then builds a final tree using tree_type for use on the logger, the initial leave one out cross validation is done to choose which id to use as the test data when building the final tree
    data_split = "build_final" #"leave_one_out" #build_final when building a final tree to use on the logger, leave_one_out when doing a grid search over parameters

    wa = False #no need to modify, used to set whether to write a new file or append when running BuildClassifier.py in a loop
    load_existing_tree = False #used during debugging

    suffix = "_" + dataset + "_" + str(upsample)

    for training_data_type in training_data_options:
        for testing_data_type in testing_data_options:

            if "random" in training_data_type:
                training_data_augmentation_type = "normal"
            else:
                training_data_augmentation_type = None

            p = Process(target=BuildClassifier.run_classifier, args=(data_type, tree_type, training_data_type, training_data_augmentation_type, testing_data_type, wa, data_split, load_existing_tree, file_dict, target_class_string, secondary_class_string, number_trees_per_setting, vary_class_weights, threshold_list, base_dir, suffix, remove_features))
            p.start()
            p.join()
            if not wa:
                wa = True

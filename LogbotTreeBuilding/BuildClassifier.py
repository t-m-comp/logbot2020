import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pydot
import datetime
import pickle
import gc

from io import StringIO
from sklearn.tree import DecisionTreeClassifier, export_graphviz
from sklearn.model_selection import KFold
from sklearn.metrics import confusion_matrix, precision_score, f1_score, recall_score
from sklearn.tree import _tree
from copy import deepcopy

from low_cost_trees import tree

import sys
import os
import subprocess
import itertools
from os import listdir
from os.path import isfile, join

def plot_confusion_matrix(cm, classes, fname,
                          normalize=False,
                          title='',
                          cmap=plt.cm.Blues):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    plt.gcf().clear()
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]

    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, cm[i, j],
                 horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black")

    plt.tight_layout()
    plt.ylabel('True class')
    plt.xlabel('Predicted class')
    plt.savefig(fname)

def encode_target(df, target_column, target_class_string, secondary_class_string, prev_targets, target_dict, binary=True):

    df_mod = df.copy()
    targets = list(df_mod[target_column].unique())
    print(targets)
    targets.sort()

    if prev_targets == None:
        prev_targets = []
    if target_dict == None:
        target_dict = {}

    if binary:
        prev_targets = [target_class_string, "others"]
        target_dict[target_class_string] = 0

        for t in targets:
            if t != target_class_string:
                target_dict[t] = 1

    else:
        if target_class_string not in prev_targets:
            prev_targets.append(target_class_string)
            target_dict[target_class_string] = 0

        if secondary_class_string is not None and secondary_class_string not in prev_targets:
            prev_targets.append(secondary_class_string)
            target_dict[secondary_class_string] = 1

        for t in targets:
            if t not in prev_targets:
                target_dict[t] = len(prev_targets)
                prev_targets.append(t)

    df_mod[target_column] = df_mod[target_column].replace(target_dict)
    return (df_mod, prev_targets, target_dict)

def visualize_tree(tree, feature_names, target_names, tree_name):
    """Create tree png using graphviz.

    Args
    ----
    tree -- scikit-learn DecsisionTree.
    feature_names -- list of feature names.
    """
    dot_data = StringIO()
    export_graphviz(tree, out_file=dot_data,
    feature_names=feature_names,
    class_names=target_names,
    filled=True, rounded=True,
    special_characters=True)
    (graph,) = pydot.graph_from_dot_data(dot_data.getvalue())
    graph.write_pdf(tree_name + ".pdf")

def tree_cost(tree, feature_names, name_map, feature_weights, weight_map):
    '''
    Gives the estimated size of code needed for feature extraction for a tree

    Parameters:
    -----------
    tree: decision tree model
        The decision tree to represent as a function
    feature_names: list
        The feature names of the dataset used for building the decision tree
    feature_weight: dict
        The size in bytes for each feature when implemented on Arduino
    '''
    tree_ = tree.tree_
    feature_names = [feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!" for i in tree_.feature]
    features_used = []

    def recurse(node, features_used):
        cost = 0
        if tree_.feature[node] != _tree.TREE_UNDEFINED:
            name = name_map[feature_names[node]]

            if name not in features_used:
                cost += feature_weights[name]
                features_used.append(name)
            for sub_name in weight_map[name]:
                if sub_name not in features_used:
                    cost += feature_weights[sub_name]
                    features_used.append(sub_name)
            cost += recurse(tree_.children_left[node], features_used)
            cost += recurse(tree_.children_right[node], features_used)
        return cost

    return recurse(0, features_used)

def tree_to_code(tree, feature_names, tree_name, target_list):
    '''
    Outputs a decision tree model as a cpp function

    Parameters:
    -----------
    tree: decision tree model
        The decision tree to represent as a function
    feature_names: list
        The feature names of the dataset used for building the decision tree
    '''

    with open(tree_name + ".cpp", "w") as code_file:
        tree_ = tree.tree_
        feature_name = [feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!" for i in tree_.feature]
        code_file.write("uint8_t tree()\n{\n")
        #print("uint8_t tree()\n{")

        def recurse(node, depth):
            indent = "  " * depth
            if tree_.feature[node] != _tree.TREE_UNDEFINED:
                name = feature_name[node]
                threshold = tree_.threshold[node]
                code_file.write(indent + "if (" + name + " <= " + str(threshold) + ")\n")
                code_file.write(indent + "{\n")
                #print(indent + "if (" + name + " <= " + str(threshold) + ")")
                #print(indent + "{")
                recurse(tree_.children_left[node], depth + 1)
                code_file.write(indent + "}\n")
                code_file.write(indent + "else\n")
                code_file.write(indent + "{\n")
                #print(indent + "}")
                #print(indent + "else")
                #print(indent + "{")
                recurse(tree_.children_right[node], depth + 1)
                code_file.write(indent + "}\n")
                #print(indent + "}")
            else:
                class_values = tree_.value[node][0]
                max_idx = 0
                for i in range(len(class_values)):
                    if class_values[i] > class_values[max_idx]:
                        max_idx = i
                code_file.write(indent + "return " + str(max_idx) + "; //" + target_list[max_idx] + "\n")
                # if tree_.value[node][0][0] < tree_.value[node][0][1]:
                #     code_file.write(indent + "return true;\n")
                #     #print(indent + "return true;")
                # else:
                #     code_file.write(indent + "return false;\n")
                #     #print(indent + "return false;")

        recurse(0, 1)
        code_file.write("}\n")
        #print("}")

def convert_gmt_to_local(a, utc_offset):
    return (24 + a + utc_offset) % 24

# def battery_level(battery_mah, run_no_cam_sec, run_w_cam_sec, slept_at_night_sec):
#     lifetime = (battery_mah / 720.0) * 21 * 60 * 60
#     non_camera_rate = 1
#     camera_rate = 21.0 / 2.0
#     sleep_rate = 1.0 / 7.0
#     return lifetime - (run_no_cam_sec * non_camera_rate) - (run_w_cam_sec * camera_rate) - (slept_at_night_sec * sleep_rate)

def battery_level(run_time_sec, record_time_sec):
    return 1 - ((run_time_sec / (20.5 * 60.0 * 60.0)) + (record_time_sec / (2.0 * 60.0 * 60.0)))

def run_classifier(data_type, tree_type, training_data_type, training_data_augmentation_type, testing_data_type, gridfile_append, data_split, load_existing_tree, file_dict, target_class_string, secondary_class_string, number_trees_per_setting, vary_class_weights, threshold_list, base_dir, suffix, remove_features=None):

    feature_weight = {
        "acc_mag_mean": 40,
        "acc_mag_mc": 102,
        "acc_mag_1c": 94,
        "acc_mag_var": 210,
        "acc_mag_energy": 60,
        "acc_mag_RMS": 246,
        "acc_mag_kurtosis": 680,
        "acc_mag_crest": 240,
        # "depth_mean": 0,
        # "depth_mean_diff": 132,
        # "depth_mc": 0,
        # "depth_var": 0,
        # "depth_energy": 0,
        # "depth_rms": 0,
        # "depth_kurt": 0,
        # "depth_crest": 0,

        "gps_manh_fpt": 144,#394,
        "gps_lat_var" : 0,
        "gps_lon_var": 0,
        "gps_lat_zc": 0,
        "gps_lon_zc": 0,
        "gps_var": 0,#0,
        "gps_zc": 204,#1254,
        "gps_avg_manh_speed": 108,#758,
        "gps_var_manh_speed": 202,#1086,
        "gps_manh_displacement": 6,#0,
        "gps_manh_distance": 114,#100,
        "gps_max_manh_displacement": 114,#64,
        "gps_avg_manh_displacement": 338,#288,
        "gps_var_manh_displacement": 448,#742,
        "gps_avg_angle": 112,#1288,
        "gps_var_angle": 234,#1410,
        "gps_avg_displacement_angle": 110,#1274,
        "gps_rotate": 1874,#1226,
        "gps_manh_dist": 828,#640,
        "gps_three_pt_angle": 1712,#1050,
        "gps_app_speed": 386,#454,
        "cos": 324,#1048,
        
        "mag_mean": 40,
        "mag_mean_diff": 132,
        "mag_min": 40,
        "mag_max": 40,
        "mag_mc": 102,
        "mag_var": 210,
        "mag_range": 0,
        "mag_energy": 60,
        "mag_kurtosis": 680,
        "mag_crest": 240,
        "mag_RMS": 246,
    }

    weight_map = {
        "acc_mag_mean" : [],
        "acc_mag_mc" : ["acc_mag_mean"],
        "acc_mag_1c" : [],
        "acc_mag_var" : [],
        "acc_mag_energy" : [],
        "acc_mag_RMS" : [],
        "acc_mag_kurtosis" : [],
        "acc_mag_crest" : ["acc_mag_RMS"],
        "depth_mean_diff": [],
        "gps_manh_fpt" : ["gps_manh_dist"],
        "gps_lat_var" : ["gps_var"],
        "gps_lon_var" : ["gps_var"],
        "gps_var" : ["gps_rotate"],
        "gps_lat_zc" : ["gps_zc"],
        "gps_lon_zc" : ["gps_zc"],
        "gps_zc" : ["gps_rotate"],
        "gps_avg_manh_speed" : ["gps_app_speed"],
        "gps_var_manh_speed" : ["gps_app_speed"],
        "gps_manh_displacement" : ["gps_manh_dist"],
        "gps_manh_distance" : ["gps_manh_dist"],
        "gps_max_manh_displacement" : ["gps_manh_dist"],
        "gps_avg_manh_displacement" : ["gps_manh_dist"],
        "gps_var_manh_displacement" : ["gps_manh_dist"],
        "gps_avg_angle" : ["gps_three_pt_angle"],
        "gps_var_angle" : ["gps_three_pt_angle"],
        "gps_avg_displacement_angle" : ["gps_three_pt_angle"],
        "gps_rotate" : ["cos"],
        "gps_manh_dist": [],
        "gps_three_pt_angle": ["cos"],
        "gps_app_speed": ["gps_manh_dist"],
        "cos": [],
        "mag_mean": [],
        "mag_mean_diff": [],
        "mag_min": [],
        "mag_max": [],
        "mag_mc": ["mag_mean"],
        "mag_var": ["mag_mean"],
        "mag_range": ["mag_min", "mag_max"],
        "mag_energy": [],
        "mag_kurtosis": [],
        "mag_crest": ["mag_RMS"],
        "mag_RMS": [],
    }

    UTC_OFFSET = 9
    DAWN = 4 + (46 / 60.0)
    DUSK = 18 + (40 / 60.0)

    MAX_TRIES = 25

    tree_model_dir = os.path.join(base_dir, "TreeModelFile" + suffix)
    tree_code_dir = os.path.join(base_dir, "TreeCode" + suffix)
    tree_pdf_dir = os.path.join(base_dir, "TreePDF" + suffix)
    cm_pdf_dir = os.path.join(base_dir, "TreeConfMatrixPdf" + suffix)

    if not os.path.exists(tree_model_dir):
        os.makedirs(tree_model_dir)
    if not os.path.exists(tree_code_dir):
        os.makedirs(tree_code_dir)
    if not os.path.exists(tree_pdf_dir):
        os.makedirs(tree_pdf_dir)
    if not os.path.exists(cm_pdf_dir):
        os.makedirs(cm_pdf_dir)

    #df = pd.read_csv(input_file, header = 0)

    target_column = "label"

    acc_features = [ "acc_mag_mean", "acc_mag_1c", "acc_mag_mc", "acc_mag_var", "acc_mag_energy", "acc_mag_kurtosis", "acc_mag_crest", "acc_mag_RMS" ] 
    gps_features = [ "gps_lat_var", "gps_lon_var", "gps_lat_zc", "gps_lon_zc", "gps_avg_angle", "gps_var_angle", "gps_avg_displacement_angle", "gps_manh_fpt", "gps_avg_manh_speed", "gps_var_manh_speed", "gps_avg_manh_displacement", "gps_var_manh_displacement", "gps_max_manh_displacement", "gps_manh_displacement", "gps_manh_distance" ]
    mag_features = ["mag_mean", "mag_mean_diff", "mag_min", "mag_max", "mag_mc", "mag_var", "mag_range", "mag_energy", "mag_kurtosis", "mag_crest", "mag_RMS"]

    if data_type == "ACC":
        column_types = {
            "animal_tag": "category",
            "acc_mag_mean": "float32",
            "acc_mag_1c": "uint8",
            "acc_mag_mc": "uint8",
            "acc_mag_var": "float32",
            "acc_mag_energy": "float32",
            "acc_mag_kurtosis": "float32",
            "acc_mag_crest": "float32",
            "acc_mag_RMS": "float32",
            "label": "category",
        }
        #older naming
        # column_types = {
        #     "birdId": "category",
        #     "accMagMean": "float32",
        #     "accMag1c": "uint8",
        #     "accMagMc": "uint8",
        #     "accMagVar": "float32",
        #     "accMagEnergy": "float32",
        #     "accMagKurtosis": "float32",
        #     "magCrest": "float32",
        #     "magRMS": "float32",
        #     "groundTruth": "category",
        # }
            
    elif data_type == "GPS":
        column_types = {
            "animal_tag": "category",
            "gps_lat_var": "float32", 
            "gps_lon_var": "float32", 
            "gps_lat_zc": "uint8", 
            "gps_lon_zc": "uint8", 
            "gps_avg_angle": "float32", 
            "gps_var_angle": "float32", 
            "gps_avg_displacement_angle": "float32", 
            "gps_manh_fpt": "uint8", 
            "gps_avg_manh_speed": "float32", 
            "gps_var_manh_speed": "float32", 
            "gps_avg_manh_displacement": "float32", 
            "gps_var_manh_displacement": "float32", 
            "gps_max_manh_displacement": "float32", 
            "gps_manh_displacement": "float32", 
            "gps_manh_distance": "float32",
            "label": "category",
        }

    elif data_type == "MAG":
        column_types = {
            "animal_tag": "category",
            "mag_mean": "float32",
            "mag_mean_diff": "float32",
            "mag_min": "float32",
            "mag_max": "float32",
            "mag_mc": "uint8",
            "mag_var": "float32",
            "mag_range": "float32",
            "mag_energy": "float32",
            "mag_kurtosis": "float32",
            "mag_crest": "float32",
            "mag_RMS": "float32",
            "label": "category",
        }
    if gridfile_append:
        wa = "a"
    else:
        wa = "w"

    with open(os.path.join(base_dir, "GridSearch" + suffix + ".csv"), wa) as grid_file:

        if not gridfile_append:
            grid_file.write("dataType,trainingDataType,testingDataType,trainingStage,treeType,maxDepth,treeNumber,costThreshold,minCost,maxCost,avgCost,weights[0],valRecall,valPrecision,valF1Score,valBinaryRecall,valBinaryPrecision,valBinaryF1Score,testRecall,testPrecision,testF1Score,testBinaryRecall,testBinaryPrecision,testBinaryF1Score,trainRecall,trainPrecision,trainF1Score,trainBinaryRecall,trainBinaryPrecision,trainBinaryF1Score\n")

        training_df = None
        testing_df = None
        validation_df = None
        training_df_splits = None
        testing_df_splits = None

        training_df_splits = {}
        testing_df_splits = {}

        name_map = {}
        name_map_strings = {}
        if data_type in ["ACC"]:
            features = acc_features
        elif data_type in ["GPS"]:
            features = gps_features
        elif data_type in ["MAG"]:
            features = mag_features


        for i, feature in enumerate(features):
            print("name_map[" + str(i) + "]: " + str(feature))
            name_map[i] = feature
            name_map_strings[feature] = feature

        training_file = file_dict[training_data_type]
        if training_data_augmentation_type is not None:
            training_augmentation_file = file_dict["normal"]
        else:
            training_augmentation_file = None
        testing_file = file_dict[testing_data_type]
        validation_file = file_dict["validation"]

        training_df = pd.read_csv(training_file, header=0, parse_dates=['timestamp'], infer_datetime_format=True)
        #training_df = training_df.dropna()
        training_df = training_df.astype(dtype=column_types)

        if remove_features != None:
            training_df[remove_features] = 0

        print("training: " + training_data_type)
        print("testing: " + testing_data_type)

        if training_augmentation_file is not None:
            training_df2 = pd.read_csv(training_augmentation_file, header=0, dtype=column_types, parse_dates=['timestamp'], infer_datetime_format=True)
            if remove_features != None:
                training_df2[remove_features] = 0
            #training_df2 = training_df2.dropna()
            training_df =  pd.concat([training_df, training_df2])
            del(training_df2)

        # print(training_data_type)
        # print(training_df.columns)
        target_dict = None
        targets = None

        training_ids = training_df.animal_tag.unique()
        for at in training_ids:
            temp_df = training_df[training_df['animal_tag'] == at]
            temp_df = temp_df[features + [target_column]]
            temp_df, targets, target_dict = encode_target(temp_df, target_column, target_class_string, secondary_class_string, targets, target_dict, binary=False)
            temp_df = temp_df.dropna()
            training_df_splits[at] = temp_df

        del(training_df)
        testing_df = pd.read_csv(testing_file, header=0, parse_dates=['timestamp'], infer_datetime_format=True)
        testing_df = testing_df.astype(dtype=column_types)
        if remove_features != None:
            testing_df[remove_features] = 0

        testing_ids = testing_df.animal_tag.unique()
        if len(training_ids) != len(testing_ids):
            raise Exception("Training and test datasets should have the same ids present.")
        for at in training_ids:
            if at not in testing_ids:
                raise Exception("Training and test datasets should have the same ids present.")

        for at in testing_ids:
            temp_df = testing_df[testing_df['animal_tag'] == at]
            temp_df = temp_df[features + [target_column]]
            temp_df, targets, target_dict = encode_target(temp_df, target_column, target_class_string, secondary_class_string, targets, target_dict, binary=False)
            temp_df = temp_df.dropna()
            testing_df_splits[at] = temp_df

        del(testing_df)
        print("targets: " + str(targets))

        validation_df = None
        validation_df = pd.read_csv(validation_file, header=0, parse_dates=['timestamp'], infer_datetime_format=True)
        validation_df = validation_df.astype(dtype=column_types)
        if remove_features != None:
            validation_df[remove_features] = 0
        validation_df = validation_df[["timestamp"] + features + [target_column]]
        validation_df = validation_df[validation_df[target_column] != "unknown"]
        validation_df[target_column] = validation_df[target_column].replace("poss_foraging", "foraging")

        validation_df, targets, target_dict = encode_target(validation_df, target_column, target_class_string, secondary_class_string, targets, target_dict, binary=False)
        validation_df = validation_df.dropna()

        max_features = int(len(features) - len(features)/4.0)

        cost_threshold_list = [500000]

        weight_list = []
        w = []
        for _ in range(len(targets)):
            w.append(1)
        weight_list.append(w)

        if vary_class_weights:
            if len(targets) == 2:
                weight_list.append([0.9, 0.1])
            elif len(targets) == 3:
                weight_list.append([0.8, 0.1, 0.1])
            elif len(targets) == 4:
                weight_list.append([0.7, 0.1, 0.1, 0.1])
            elif len(targets) == 5:
                weight_list.append([0.6, 0.1, 0.1, 0.1, 0.1])
            elif len(targets) > 5:
                print("Need to update code for this many target classes")

        cost_threshold_list = threshold_list

        max_tree_depth = 5
        training_stages = ["cross_validate"]

        if data_split == "build_final": #first run cross_validation to pick hold out set for final tree, then build and test the candidate for use on logbot
            training_stages = ["cross_validate", "build_final"]

        if load_existing_tree:
            training_stages = ["load_tree"]
            number_trees_per_setting = 1
            cost_threshold_list = [0]
            weight_list = [weight_list[0]]
            tesing_ids = [testing_ids[0]]

        f1_dictionary = {}

        for cost_threshold in cost_threshold_list:
            for weights in weight_list:
                for training_stage in training_stages:
                    print("starting new iteration:")
                    print(training_stage)
                    print(str(cost_threshold))
                    print(str(weights[0]))
                    if training_stage == "build_final":
                        print("choosing test animals for final tree:")
                        f1_list = []
                        for ti in testing_ids:
                            print("f1 scores from cross validation: ")
                            print(ti + ": " + str(f1_dictionary[cost_threshold][weights[0]][ti]))
                            f1_list.append((f1_dictionary[cost_threshold][weights[0]][ti], ti))
                        f1_list.sort()
                        med_ids = []
                        med_id_count = 1
                        print("animals chosen:")
                        if len(f1_list) > 3:
                            med_id_count = max(len(f1_list) // 6, med_id_count)
                            mid = len(f1_list) // 2
                            if med_id_count//2 == 0:
                                print(f1_list[mid][1])
                                med_ids.append(f1_list[mid][1])
                            else:
                                for i in range(mid-(med_id_count//2), mid+(med_id_count//2)):
                                    print(f1_list[i][1])
                                    med_ids.append(f1_list[i][1])
                        elif len(f1_list) > 1:
                            print(f1_list[1][1])
                            med_ids = [f1_list[1][1]]
                        else:
                            print("Only one animal, so using that animal as test and train")
                            med_ids = [f1_list[0][1]]

                    if training_stage == "build_final":
                        num_trees = number_trees_per_setting
                    else:
                        num_trees = 1

                    for tree_number in range(num_trees):

                        test_ground_truth_list = []
                        test_prediction_list = []

                        min_cost = 50000
                        max_cost = 0
                        avg_cost = []
                        split_count = 0

                        for testId in testing_ids:

                            if training_stage == "cross_validate":
                                if cost_threshold not in f1_dictionary:
                                    f1_dictionary[cost_threshold] = {}
                                if weights[0] not in f1_dictionary[cost_threshold]:
                                    f1_dictionary[cost_threshold][weights[0]] = {}
                                if testId not in f1_dictionary[cost_threshold][weights[0]]:
                                    f1_dictionary[cost_threshold][weights[0]][testId] = 0
                            else:
                                if split_count > 0:
                                    break
                                else:
                                    split_count += 1

                            train = None
                            test = None
                            
                            if len(training_ids) == 1:
                                print("Only 1 id found, using same data as both train and test data")

                            if training_stage != "load_tree":
                                if training_stage == "cross_validate":
                                    test = testing_df_splits[testId].copy()
                                    train = None
                                    #pd.concat([df_a, df_b])
                                    for nextId in training_ids:
                                        if nextId != testId or len(training_ids) == 1:
                                            if train is None:
                                                train = training_df_splits[nextId].copy()
                                            else:
                                                train = pd.concat([train, training_df_splits[nextId].copy()])
                                else:
                                    for nextId in testing_ids:
                                        if nextId in med_ids:
                                            if test is None:
                                                test = testing_df_splits[nextId].copy()
                                            else:
                                                test = pd.concat([test, testing_df_splits[nextId].copy()])
                                    for nextId in training_ids:
                                        if nextId not in med_ids or len(f1_list) == 1:
                                            if train is None:
                                                train = training_df_splits[nextId].copy()
                                            else:
                                                train = pd.concat([train, training_df_splits[nextId].copy()])

                                X_train = train[features]
                                y_train = train[target_column]

                                X_test = test[features]
                                y_test = test[target_column]

                            X_validate = validation_df[features]
                            y_validate = validation_df[target_column]
                            t_validate = validation_df["timestamp"]

                            w = {}
                            for i in range(len(targets)):
                                # print("w: " + str(targets[i]) + " " + str(i))
                                w[i] = weights[i]

                            #default tree is used when performing cross validation prior to training final tree so that same validation set will be used with each weight. When choosing a tree from results from several different costs, it is otherwise hard to compare between costs because different validation sets were used. For example, 10 trees from a single validation set may outpeform all other trees, but may not be the best trees.
                            if training_stage == "load_tree":
                                dt = pickle.load(open("tree.pkl", 'rb'))
                            else:
                                if tree_type == "default_tree" or (training_stage == "cross_validate" and data_split == "build_final"):
                                    print("Building tree using sklearn default algorithm")
                                    dt = DecisionTreeClassifier(max_depth=max_tree_depth, class_weight=w)

                                    y_set = set(list(y_train))
                                    y_set_strings = set()
                                    for y_i in y_set:
                                        for k in target_dict:
                                            if target_dict[k] == y_i:
                                                y_set_strings.add(k)
                                    if (y_set_strings != set(targets)):
                                        raise ValueError("Each iteration of training needs all classes represented in the training data")

                                    dt.fit(X_train, y_train)

                                elif tree_type == "rf_tree":

                                    print("Building tree using low_cost_tree algorithm")

                                    best_tree = None
                                    best_f1 = 0
                                    smallest_tree_cost = 50000

                                    for i in range(MAX_TRIES):
                                        # print("Training weighted tree")
                                        dt = tree.WeightedDecisionTreeClassifier(max_depth=max_tree_depth,
                                                                                class_weight=w,
                                                                                feature_weight=feature_weight,
                                                                                name_map=name_map,
                                                                                weight_map=weight_map,
                                                                                splitter="weighted",
                                                                                max_features=max_features)
                                        dt.fit(X_train, y_train)
                                        curr_cost = tree_cost(dt, features, name_map_strings, feature_weight, weight_map)
                                        # print("curr cost: " + str(curr_cost))
                                        if curr_cost < cost_threshold:
                                            val_pred = dt.predict(X_train)
                                            val_f1 = f1_score(y_train, val_pred, average="macro")

                                            if val_f1 > best_f1:
                                                best_f1 = val_f1
                                                best_tree = deepcopy(dt)
                                        elif best_f1 == 0 and curr_cost < smallest_tree_cost:
                                            smallest_tree_cost = curr_cost
                                            best_tree = deepcopy(dt)
                                    dt = best_tree
                                    # print("best tree f1 score: " + str(best_f1))
                                else:
                                    raise Exception("Unknown decision tree type")

                                y_pred = dt.predict(X_test)

                            y_val_pred = dt.predict(X_validate)
                            y_train_pred = dt.predict(X_train)

                            curr_cost = tree_cost(dt, features, name_map_strings, feature_weight, weight_map)
                            print("curr_cost: " + str(curr_cost))
                            if curr_cost < min_cost:
                                min_cost = curr_cost
                            if curr_cost > max_cost:
                                max_cost = curr_cost
                            avg_cost.append(curr_cost)

                            if training_stage == "load_tree":
                                t_validate = t_validate.tolist()
                                y_validate = y_validate.tolist()
                            else:

                                if data_split == "leave_one_out" or training_stage == "build_final":
                                    tree_name =  data_type + "_" + training_data_type + "_" + testing_data_type +  "_" + tree_type + "_treeNbr" + str(tree_number) + "_costThr" + str(cost_threshold) + "_weight" + \
                                                            str(weights[0]) + "_" + testId + "_cost" + str(curr_cost)
                                    visualize_tree(dt, features, targets, os.path.join(tree_pdf_dir, tree_name))
                                    tree_to_code(dt, features, os.path.join(tree_code_dir, tree_name), targets)
                                    decision_tree_model_pkl = open(os.path.join(tree_model_dir, tree_name + ".pkl"), 'wb')
                                    pickle.dump(dt, decision_tree_model_pkl)
                                    decision_tree_model_pkl.close()
                                else:
                                    tree_name =  data_type + "_" + training_data_type + "_" + testing_data_type +  "_" + tree_type + "_treeNbrNA_costThr" + str(cost_threshold) + "_weight" + \
                                                            str(weights[0]) + "_" + testId + "_cost" + str(curr_cost)
                                    visualize_tree(dt, features, targets, os.path.join(tree_pdf_dir, tree_name))
                                    tree_to_code(dt, features, os.path.join(tree_code_dir, tree_name), targets)
                                    decision_tree_model_pkl = open(os.path.join(tree_model_dir, tree_name + ".pkl"), 'wb')
                                    pickle.dump(dt, decision_tree_model_pkl)
                                    decision_tree_model_pkl.close()

                                test_ground_truth_list = list(test_ground_truth_list) + list(y_test)
                                test_prediction_list = list(test_prediction_list) + list(y_pred)

                                temp_f1_score = f1_score(y_test, y_pred, average="macro")

                                if temp_f1_score > f1_dictionary[cost_threshold][weights[0]][testId]:
                                    f1_dictionary[cost_threshold][weights[0]][testId] = temp_f1_score

                        if training_stage != "load_tree":
                            index = 0

                            test_recall = recall_score(test_ground_truth_list, test_prediction_list, average="macro")
                            test_precision = precision_score(test_ground_truth_list, test_prediction_list, average="macro")
                            test_f1_score = f1_score(test_ground_truth_list, test_prediction_list, average="macro")

                            #val scores use data collected from logbot as validation data
                            val_recall = recall_score(y_validate, y_val_pred, average="macro")
                            val_precision = precision_score(y_validate, y_val_pred, average="macro")
                            val_f1_score = f1_score(y_validate, y_val_pred, average="macro")

                            binary_val_prediction_list = []
                            binary_val_ground_truth_list = []
                            gt_count = 0
                            pr_count = 0

                            y_validate = y_validate.tolist()
                            y_val_pred = y_val_pred.tolist()

                            for k in range(len(y_validate)):
                                if y_val_pred[k] == 0:
                                    binary_val_prediction_list.append(1)
                                    pr_count += 1
                                else:
                                    binary_val_prediction_list.append(0)
                                if y_validate[k] == 0:
                                    binary_val_ground_truth_list.append(1)
                                    gt_count += 1
                                else:
                                    binary_val_ground_truth_list.append(0)

                            val_binary_recall = recall_score(binary_val_ground_truth_list, binary_val_prediction_list, average="binary")
                            val_binary_precision = precision_score(binary_val_ground_truth_list, binary_val_prediction_list, average="binary")
                            val_binary_f1_score = f1_score(binary_val_ground_truth_list, binary_val_prediction_list, average="binary")

                            train_recall = recall_score(y_train, y_train_pred, average="macro")
                            train_precision = precision_score(y_train, y_train_pred, average="macro")
                            train_f1_score = f1_score(y_train, y_train_pred, average="macro")

                            binary_train_prediction_list = []
                            binary_train_ground_truth_list = []
                            gt_count = 0
                            pr_count = 0

                            y_train = y_train.tolist()
                            y_train_pred = y_train_pred.tolist()

                            for k in range(len(y_train)):
                                if y_train_pred[k] == 0:
                                    binary_train_prediction_list.append(1)
                                    pr_count += 1
                                else:
                                    binary_train_prediction_list.append(0)
                                if y_train[k] == 0:
                                    binary_train_ground_truth_list.append(1)
                                    gt_count += 1
                                else:
                                    binary_train_ground_truth_list.append(0)

                            train_binary_recall = recall_score(binary_train_ground_truth_list, binary_train_prediction_list, average="binary")
                            train_binary_precision = precision_score(binary_train_ground_truth_list, binary_train_prediction_list, average="binary")
                            train_binary_f1_score = f1_score(binary_train_ground_truth_list, binary_train_prediction_list, average="binary")

                            if data_split == "leave_one_out" or training_stage == "build_final":
                                tree_name =  data_type + "_" + training_data_type + "_" + testing_data_type + "_" + tree_type + "_treeNbr" + str(tree_number) + "_costThr" + str(cost_threshold) + "_weight" + \
                                                        str(weights[0]) + "_" + testId + "_cost" + str(curr_cost)
                                test_cm = confusion_matrix(test_ground_truth_list, test_prediction_list)
                                plot_confusion_matrix(test_cm, targets, os.path.join(cm_pdf_dir, tree_name + ".pdf"))
                                val_cm = confusion_matrix(y_validate, y_val_pred)
                                plot_confusion_matrix(val_cm, targets, os.path.join(cm_pdf_dir, tree_name + "_val" + ".pdf"))
                                train_cm = confusion_matrix(y_train, y_train_pred)
                                plot_confusion_matrix(train_cm, targets, os.path.join(cm_pdf_dir, tree_name + "_train" + ".pdf"))
                            else:
                                tree_name =  data_type + "_" + training_data_type + "_" + testing_data_type + "_" + tree_type + "_treeNbrNA_costThr" + str(cost_threshold) + "_weight" + \
                                                        str(weights[0]) + "_" + testId + "_cost" + str(curr_cost)
                                test_cm = confusion_matrix(test_ground_truth_list, test_prediction_list)
                                plot_confusion_matrix(test_cm, targets, os.path.join(cm_pdf_dir, tree_name + ".pdf"))
                                val_cm = confusion_matrix(y_validate, y_val_pred)
                                plot_confusion_matrix(val_cm, targets, os.path.join(cm_pdf_dir, tree_name + "_val" + ".pdf"))
                                train_cm = confusion_matrix(y_train, y_train_pred)
                                plot_confusion_matrix(train_cm, targets, os.path.join(cm_pdf_dir, tree_name + "_train" + ".pdf"))

                            binary_test_prediction_list = []
                            binary_test_ground_truth_list = []
                            gt_count = 0
                            pr_count = 0

                            for k in range(len(test_prediction_list)):
                                if test_prediction_list[k] == 0:
                                    binary_test_prediction_list.append(1)
                                    pr_count += 1
                                else:
                                    binary_test_prediction_list.append(0)
                                if test_ground_truth_list[k] == 0:
                                    binary_test_ground_truth_list.append(1)
                                    gt_count += 1
                                else:
                                    binary_test_ground_truth_list.append(0)
                            
                            test_binary_recall = recall_score(binary_test_ground_truth_list, binary_test_prediction_list, average="binary")
                            test_binary_precision = precision_score(binary_test_ground_truth_list, binary_test_prediction_list, average="binary")
                            test_binary_f1_score = f1_score(binary_test_ground_truth_list, binary_test_prediction_list, average="binary")

                            avg_cost = sum(avg_cost) / float(len(avg_cost))

                            tn = str(tree_number)
                            if training_stage == "cross_validate":
                                tn = "NA" 

                            grid_file.write(data_type + "," + training_data_type + "," + testing_data_type + "," + training_stage + "," + tree_type + "," + str(max_tree_depth) + "," + tn + "," + str(cost_threshold) +
                                    "," + str(min_cost) + "," + str(max_cost) + "," + str(avg_cost) + "," + str(weights[0]) + "," + str(val_recall) + "," + str(val_precision) + "," + str(val_f1_score) +
                                    "," + str(val_binary_recall) + "," + str(val_binary_precision) + "," + str(val_binary_f1_score) + "," + str(test_recall) + "," + str(test_precision) + "," + str(test_f1_score) + "," + str(test_binary_recall) +
                                    "," + str(test_binary_precision) + "," + str(test_binary_f1_score)  + "," + str(train_recall) + "," + str(train_precision) + "," + str(train_f1_score) + "," + str(train_binary_recall) +
                                    "," + str(train_binary_precision) + "," + str(train_binary_f1_score) + "\n")

if __name__ == '__main__':
    pass

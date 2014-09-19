#!/usr/bin/env python
from collections import defaultdict

import numpy as np
import sys
import re
from datetime import datetime, timedelta
import os

import IPython

CONFIG = {'verbose': False}

class NumericData:
    def __init__(self, data = None):
        if not data: data = []
        self.data = data

    def add(self, d):
        self.data.append(d)

    def mean(self):
        if len(self.data) > 0:
            return sum(self.data)/float(len(self.data))
        return None

    def sd(self):
        m = self.mean
        if m is None:
            return None
        return np.std(np.array(self.data))
    
class FileGroupProcessor():
    def __init__(self):
        group_names = ['bsp_bathroom', 'bsp_kitchen']
#         group_names = ['bsp_bathroom'] + ['sampling_bathroom/sampling{0}'.format(i) for i in [10,50,100,200]] + \
#                       ['bsp_kitchen'] + ['sampling_kitchen/sampling{0}'.format(i) for i in [10,50,100,200]]
                      
        self.fgs = list()
        for group_name in group_names:
            self.fgs.append(FileGroup([group_name + '/' + f for f in os.listdir(group_name) if f.count('.txt')], group_name))
            
    def latex_table(self):
        latex_str = ('\\begin{table*}[t] \n'
                     '\\centering \n'
                     '\\begin{tabular}{l || c c c c c | c c c c c} \n'
                     '& \multicolumn{5}{c|}{Bathroom} & \multicolumn{5}{c}{Kitchen} \\\\ \n'
                     '& Trajectory & \multicolumn{4}{c|}{Sampling}& Trajectory & \multicolumn{4}{c}{Sampling}\\\\ \n'
                     '& Optimization & 10   & 50   & 100  & 200  & Optimization & 10   & 50   & 100  & 200  \\\\ \n')

        latex_str += '\\hline & \multicolumn{5}{c|{} \\\\ \n'
        latex_str += 'Total number of objects & ' + ' & '.join([str(fg.num_objects) for fg in self.fgs]) + ' \\\\ \n'
        
        latex_str += '\\hline & \multicolumn{5}{c|{} \\\\ \n'
        latex_str += 'Objects successfully grasped (\%) & ' + ' & '.join([str(fg.objects_successfully_grasped_pct) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Objects missed (\%) & ' + ' & '.join(['{0:.4}'.format(fg.objects_missed_pct) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Objects dropped (\%) & ' + ' & '.join(['{0:.4}'.format(fg.objects_dropped_pct) for fg in self.fgs]) + ' \\\\ \n'

        latex_str += '\\hline & \multicolumn{5}{c|{} \\\\ \n'
        latex_str += 'Avg. time to plan (s) & ' + \
                     ' & '.join(['{0:.4} $\pm$ {1:.4}'.format(fg.avg_plan_time_s, fg.sd_plan_time_s) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Avg. time to see handle (s) & ' + \
                     ' & '.join(['{0:.4} $\pm$ {1:.4}'.format(fg.avg_time_to_first_handle_s, fg.sd_time_to_first_handle_s) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Avg. time to grasp attempt (s) & ' + \
                     ' & '.join(['{0:.4} $\pm$ {1:.4}'.format(fg.avg_time_to_grasp_attempt_s, fg.sd_time_to_grasp_attempt_s) for fg in self.fgs]) + ' \\\\ \n'

        latex_str += '\\hline & \multicolumn{5}{c|{} \\\\ \n'
        latex_str += 'Avg. run time (s) & ' + \
                     ' & '.join(['{0:.4} $\pm$ {1:.4}'.format(fg.avg_run_time_s, fg.sd_run_time_s) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Occluded Region time (\%) & ' + \
                     ' & '.join(['{0:.4}'.format(fg.occluded_region_time_pct) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Planning time (\%) & ' + \
                     ' & '.join(['{0:.4}'.format(fg.planning_time_pct) for fg in self.fgs]) + ' \\\\ \n'
        latex_str += 'Exploring time (\%) & ' + \
                     ' & '.join(['{0:.4}'.format(fg.exploring_time_pct) for fg in self.fgs]) + ' \\\\ \n'
                     
        latex_str += '\\hline & \multicolumn{5}{c|{} \\\\ \n'
        latex_str += 'Experiments failed (\%) & ' + \
                     ' & '.join(['{0:.4}'.format(fg.experiments_failed_pct) for fg in self.fgs]) + ' \\\\ \n'

        latex_str += ('\end{tabular} \n'
                     '\\caption{\\TODO{}} \n'
                     '\\label{table:results} \n'
                     '\\end{table*} \n')

        return latex_str
            
    def summaries(self):
        s = ''
        for fg in self.fgs:
            s += fg.summary() + '\n'
            
        return s

class FileGroup:
    def __init__(self, filenames, name):
        self.files = []
        self.runs = []
        self.name = name
        self.grasps_attempted = 0
        self.successful_grasps = 0
        self.missed_grasps = 0
        self.drops = 0
        self.experiment_times = NumericData()
        self.run_times = NumericData()
        self.first_handle_times = NumericData()
        self.total_time_runs = timedelta(0)
        self.total_time_experiments = timedelta(0)

        if "bathroom" in self.name:
            self.objects_per_experiment = 2
        elif "kitchen" in self.name:
            self.objects_per_experiment = 3

        i = 0
        for filename in filenames:
            current_file = File("{0}, experiment {1}".format(name, i), filename)
            i += 1
            if len(current_file.lines) > 1:
                self.files.append(current_file)
                for run in current_file.split_runs():
                    if len(run.lines) > 1:
                        self.runs.append(run)

       
        self.calculate_group_statistics()
        self.calculate_experiment_level_statistics()
        
    
    def calculate_group_statistics(self):
        for run in self.runs:
            proc = LogfileProcessor(run)
            self.grasps_attempted += proc.grasps_attempted
            self.successful_grasps += proc.successful_grasps
            self.total_time_runs += proc.total_time()
            self.run_times.add(proc.total_time().total_seconds())
            self.missed_grasps += proc.missed_grasps
            self.drops += proc.drops
            try:
                self.first_handle_times.add(proc.time_to_first_handle().total_seconds())
            except:
                pass

    def calculate_experiment_level_statistics(self):
        for f in self.files:
            proc = LogfileProcessor(f)
            self.total_time_experiments += proc.total_time()
            self.experiment_times.add(proc.total_time().total_seconds())
            
    @property
    def num_objects(self):
        return len(self.files) * self.objects_per_experiment
            
    @property
    def objects_successfully_grasped_pct(self):
        return 100.0 * self.successful_grasps / self.num_objects
            
    @property
    def objects_missed_pct(self):
        return 100.0 * self.missed_grasps / self.num_objects
    
    @property
    def objects_dropped_pct(self):
        return 100.0 * self.drops / self.num_objects
    
    @property
    def avg_plan_time_s(self):
        return -1.0 # TODO
    @property
    def sd_plan_time_s(self):
        return -1.0 # TODO
    
    @property
    def avg_time_to_first_handle_s(self):
        return self.first_handle_times.mean()
    @property
    def sd_time_to_first_handle_s(self):
        return self.first_handle_times.sd()
    
    @property
    def avg_time_to_grasp_attempt_s(self):
        return -1.0 # TODO
    @property
    def sd_time_to_grasp_attempt_s(self):
        return -1.0 # TODO
    
    @property
    def avg_run_time_s(self):
        return self.run_times.mean()
    @property
    def sd_run_time_s(self):
        return self.run_times.sd()
    
    @property
    def occluded_region_time_pct(self):
        return -1.0 # TODO
    
    @property
    def planning_time_pct(self):
        return -1.0 # TODO
    
    @property
    def exploring_time_pct(self):
        return -1.0 # TODO
    
    @property
    def grasping_time_pct(self):
        return -1.0 # TODO
    
    @property
    def experiments_failed_pct(self):
        return -1.0 # TODO
            
#     def average_experiment_time(self):
#         return timedelta(seconds=self.experiment_times.mean())
# 
#     def sd_experiment_time(self):
#         return timedelta(seconds=self.experiment_times.sd())
            
    def summary(self):
        result = "Summary for group: {0}\n".format(self.name)
        
        result += 'Total number of objects: {0}\n\n'.format(self.num_objects)
        
        result += 'Objects successfully grasped (%): {0:.4}\n'.format(self.objects_successfully_grasped_pct)
        result += 'Objects missed (%): {0:.4}\n'.format(self.objects_missed_pct)
        result += 'Objects dropped (%): {0:.4}\n\n'.format(self.objects_dropped_pct)
        
        result += 'Avg. time to plan (s): {0:.4} +- {1:.4}\n'.format(self.avg_plan_time_s, self.sd_plan_time_s)
        result += 'Avg. time to see handle (s): {0:.4} +- {1:.4}\n'.format(self.avg_time_to_first_handle_s, self.sd_time_to_first_handle_s)
        result += 'Avg. time to grasp attempt (s): {0:.4} +- {1:.4}\n\n'.format(self.avg_time_to_grasp_attempt_s, self.sd_time_to_grasp_attempt_s)
        
        result += 'Avg. run time (s): {0:.4} +- {1:.4}\n'.format(self.avg_run_time_s, self.sd_run_time_s)
        result += 'Occluded Region time (%): {0}\n'.format(self.occluded_region_time_pct)
        result += 'Planning time (%): {0}\n'.format(self.planning_time_pct)
        result += 'Exploring time (%): {0}\n'.format(self.exploring_time_pct)
        result += 'Grasping time (%): {0}\n\n'.format(self.grasping_time_pct)
        
        result += 'Experiments failed (%): {0}\n\n\n'.format(self.experiments_failed_pct)

#         result += "Ran {0} experiments, for a total of {1} runs.\n".format(len(self.files), len(self.runs))
#         result += "Average experiment time: {0}\n".format(self.average_experiment_time())
#         result += "Standard deviation: {0}\n".format(self.sd_experiment_time())
                    
        return result
        

class File:
    def __init__(self, name, file_name=None):
        self.lines = []
        self.runs = []
        self.name = name
        if file_name:
            with open(file_name, 'r') as file:
                for line in file:
                    if line.count("--") == 0:
                        self.lines.append(line)

    def __getitem__(self, i):
        return self.lines[i]

    def addline(self, line):
        self.lines.append(line)

    def getlines(self):
        return self.lines

    def split_runs(self):
        if not self.runs:
            run_index = -1
            i = 0
            current = File(name="Run {0}".format(i))
            for line in self.lines:
                i += 1
                if line.count("start kinfu_reset") > 0 or i == len(self.lines):
                    self.runs.append(current)
                    run_index += 1
                    current = File("Run {0}".format(run_index))
                current.addline(line)

        return self.runs
        

class LogfileProcessor:
    def __init__(self, infile):
        
            self.start = defaultdict(list)
            self.end = defaultdict(list)
            self.info = defaultdict(list)
            self.name = infile.name
            self.first_handle_time = None
            self.zero_weights = []
            self.nonzero_weights = []
            self.successful_grasps = 0
            self.grasps_attempted = 0
            self.missed_grasps = 0
            self.drops = 0
        
            if CONFIG['verbose']:
                print "processing {}".format(self.name)

            for line in infile.getlines():
                self.process_line(line)

            self.start_of_experiment = self.split_line(infile[0])['time']
            self.end_of_experiment = self.split_line(infile[-1])['time']

            self.total_times = dict()
            self.post_process()
            
    
    @staticmethod
    def split_line(line):
        search_results = re.search('^\[(.*)\]\s(\w+)\s(\w+)\s?(.*)', line) # extract time from line
        if search_results:
            time = LogfileProcessor.split_time(search_results.group(1))
            info = search_results.group(4) # all stuff after name
            name = search_results.group(3) # TODO: get name of segment start
            return {'time': time, 'info': info, 'name': name}
        else:
            return None

    @staticmethod
    def get_handles(line):
        search_results = re.search('^\[(.*)\]\s(\w+)\s([0-9]+)', line)
        if search_results:
            time = LogfileProcessor.split_time(search_results.group(1))
            count = int(search_results.group(3))
            return {'time': time, 'count': count}
        else:
            return None

    @staticmethod
    def split_time(timestring):
        timestringlist = re.split(':|-|\s|\.', timestring)
        time = datetime(*[int(s) for s in timestringlist])
        return time
    
    @staticmethod
    def count_weights(line):
        search_results = re.search('^\[(.*)\]\s(kinfu)\s(\w+)\s(weights)\s([0-9]+)', line)
        if search_results:
            time = LogfileProcessor.split_time(search_results.group(1))
            zero = search_results.group(3) == "zero" # all stuff after name
            count = int(search_results.group(5)) # TODO: get name of segment start
            return {'time': time, 'zero': zero, 'count': count}
        else:
            return None

            

    def process_line(self, line):
        result = self.split_line(line)
        if result:
            if line.count("missed grasp") > 0:
                self.missed_grasps += 1
            if line.count('start') > 0:
                self.start[result['name']].append(result['time'])
                if line.count("execute_grasp_trajectory") > 0:
                    self.grasps_attempted += 1
            elif line.count('end') > 0:
                self.end[result['name']].append(result['time'])
            elif line.count('handles') and self.get_handles(line)['count'] > 0 and not self.first_handle_time:
                self.first_handle_time = self.get_handles(line)['time']
            elif line.count('weights') > 0:
                count = self.count_weights(line)
                if count['zero']:
                    self.zero_weights.append(count['count'])
                else:
                    self.nonzero_weights.append(count['count'])
            elif line.count("successful grasp") > 0:
                self.successful_grasps += 1
            elif line.count("dropped object") > 0:
                self.drops += 1

            
        
            
    def post_process(self):
        for name, start_list in self.start.items():
            end_list = self.end[name]
            try:
                assert(len(start_list) == len(end_list))
            except AssertionError:
                if CONFIG['verbose']:
                    print "[warning: {0} was incomplete at the end of the experiment]".format(name)
                    print "[Assuming {0} lasted until end of experiment]".format(name)
                end_list.append(self.end_of_experiment)
            self.total_times[name] = np.sum(np.array(end_list) - np.array(start_list))

    
    def total_time(self):
        return sum(self.total_times.values(), timedelta(0))

    def experiment_duration(self):
        return self.end_of_experiment - self.start_of_experiment
        
    def time_to_first_handle(self):
        return self.first_handle_time - self.start_of_experiment

            
    def summary(self):
        result = "Summary of {0}:\n".format(self.name)
        timesum = self.total_time()
        for name, time in self.total_times.items():
            result += "{0}: {1} ({2:.3g}%)\n".format(name, time, 100 * time.total_seconds() / timesum.total_seconds())
    
        result += "total: {0}\n".format(timesum)
        if self.start_of_experiment:
            result += "experiment total time: {0}\n".format(self.experiment_duration())

        result += "\nTime before first handle found: {0}".format(self.time_to_first_handle())
        for i in range(len(self.nonzero_weights)):
            total_voxels = float(self.nonzero_weights[i] + self.zero_weights[i])
            result += "\nCount of nonzero weight voxels: {0} ({1:.3g}%)\n".format(self.nonzero_weights[i], 100 * self.nonzero_weights[i] / total_voxels)
            result += "Count of zero weight voxels: {0} ({1:.3g}%)\n".format(self.zero_weights[i], 100 * self.zero_weights[i] / total_voxels)
            if self.grasps_attempted > 0:
                result += "Successful grasps: {0}/{1} ({2}%)\n".format(self.successful_grasps, self.grasps_attempted, 100.0 * self.successful_grasps / self.grasps_attempted)
            else:
                result += "No grasps attempted.\n"
        return result


def process_all_files():
    group_names = ['bsp_bathroom']
#     group_names = ['bsp_bathroom'] + ['sampling_bathroom/sampling{0}'.format(i) for i in [10,50,100,200]] + \
#                   ['bsp_kitchen'] + ['sampling_kitchen/sampling{0}'.format(i) for i in [10,50,100,200]]
                  
    file_groups = list()
    for group_name in group_names:
        file_groups.append(FileGroup([group_name + '/' + f for f in os.listdir(group_name) if f.count('.txt')], group_name))
        
    for fg in file_groups:
        print(fg.summary())
        
            
if __name__ == '__main__':
    fg_proc = FileGroupProcessor()
    print(fg_proc.summaries())
#     print(fg_proc.latex_table())
    
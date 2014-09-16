#!/usr/bin/env python
from collections import defaultdict

import numpy as np
import sys
import re
from datetime import datetime, timedelta

CONFIG = {'verbose': False}

class File:
    def __init__(self, name, file_name=None):
        self.lines = []
        self.runs = []
        self.name = name

        if file_name:
            with open(file_name, 'r') as file:
                for line in file:
                    if line.count("------") == 0:
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
            self.last_zero_weights = None
            self.last_nonzero_weights = None
        
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
            if line.count('start') > 0:
                self.start[result['name']].append(result['time'])
            elif line.count('end') > 0:
                self.end[result['name']].append(result['time'])
            elif line.count('handles') and self.get_handles(line)['count'] > 0 and not self.first_handle_time:
                self.first_handle_time = self.get_handles(line)['time']
            elif line.count('weights'):
                count = self.count_weights(line)
                if count['zero']:
                    self.last_zero_weights = count['count']
                else:
                    self.last_nonzero_weights = count['count']
            
        
            
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
            
    def summary(self):
        result = "Summary of {0}:\n".format(self.name)
        timesum = sum(self.total_times.values(), timedelta(0))
        for name, time in self.total_times.items():
            result += "{0}: {1} ({2:.3g}%)\n".format(name, time, 100 * time.total_seconds() / timesum.total_seconds())
    
        result += "total: {0}\n".format(timesum)
        if self.start_of_experiment:
            result += "experiment total time: {0}\n".format(self.end_of_experiment - self.start_of_experiment)

        result += "\nTime before first handle found: {0}".format(self.first_handle_time - self.start_of_experiment)
        total_voxels = float(self.last_nonzero_weights + self.last_zero_weights)
        result += "\nCount of nonzero weight voxels: {0} ({1:.3g}%)\n".format(self.last_nonzero_weights, 100 * self.last_nonzero_weights / total_voxels)
        result += "Count of zero weight voxels: {0} ({1:.3g}%)\n".format(self.last_zero_weights, 100 * self.last_zero_weights / total_voxels)
        return result


        
            
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Error: no file specified"
        sys.exit(1)

    file_name = sys.argv[1]
    CONFIG["verbose"] = sys.argv.count("-v") > 0
    CONFIG["total"] = sys.argv.count("-t") > 0

    infile = File("all runs", file_name)
    proc = LogfileProcessor(infile)
    runs = infile.split_runs()
    run_procs = []
    for run in runs[1:]:
        run_procs.append(LogfileProcessor(run))

    print
    print proc.summary()
    if not CONFIG["total"]:
        for run_proc in run_procs:
            print
            print run_proc.summary()
    print

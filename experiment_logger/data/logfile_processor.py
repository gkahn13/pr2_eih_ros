#!/usr/bin/env python
from collections import defaultdict

import numpy as np
import sys
import re
from datetime import datetime, timedelta

class LogfileProcessor:
    def __init__(self, file_name):
        with open(file_name, 'r') as file:
            self.start = defaultdict(list)
            self.end = defaultdict(list)
        
            for line in file:
                self.process_line(line)
            
            self.total_times = dict()
            self.post_process()
            
    def process_line(self, line):
        search_results = re.search('^\[(.*)\]\s(\w+)\s(\w+)\s?(.*)', line) # extract time from line
        if search_results:
            timestringlist = re.split(':|-|\s|\.', search_results.group(1))
            time = datetime(*[int(s) for s in timestringlist])
            info = search_results.group(2) # all stuff after time
        
            if line.count('Beginning') > 0:
                self.start_of_experiment = time
            elif line.count('End') > 0:
                self.end_of_experiment = time
            elif line.count('start') > 0:
                name = search_results.group(3) # TODO: get name of segment start
                self.start[name].append(time)
            elif line.count('end') > 0:
                name = search_results.group(3) # TODO: get name of segment start
                self.end[name].append(time)
            
    def post_process(self):
        for name, start_list in self.start.items():
            end_list = self.end[name]
            try:
                assert(len(start_list) == len(end_list))
            except AssertionError:
                print "{0} was incomplete at the end of the experiment".format(name)
                print "Assuming {0} lasted until end of experiment".format(name)
                end_list.append(self.end_of_experiment)
            self.total_times[name] = np.sum(np.array(end_list) - np.array(start_list))
            
    def __str__(self):
        result = "Summary:\n"
        timesum = timedelta(0)
        for name, time in self.total_times.items():
            result += "{0}: {1}\n".format(name, time)
            timesum += time
    
        result += "total: {0}\n".format(timesum)
        result += "experiment total time: {0}\n".format(self.end_of_experiment - self.start_of_experiment)
        return result
            
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Error: no file specified"
        sys.exit(1)

    file_name = sys.argv[1]
    proc = LogfileProcessor(file_name)
    print
    print proc

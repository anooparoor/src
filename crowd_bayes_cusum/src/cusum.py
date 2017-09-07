# implement cusum algorithm for online change detection in crowded environments
import math
import matplotlib.pyplot as plt
import numpy as np

# function that takes a sequence of samples x1,x2,x3 and returns change point 
class Cusum:
    def __init__(self, change, threshold):
        self.cusum_log_likelihood = 0.0
	self.min_cusum_log_likelihood = 0.0
	self.sample_sum = 0.0
	self.sample_count = 0
	self.change_point = -1
	# two input parameters
	self.change_magnitude = change # expected change in mean is a parameter
	self.decision_threshold = threshold # how confident are you before declaring change

    def poisson_log_likelihood(self, param_before_change, param_after_change, sample):
	l1 = param_after_change
	l0 = param_before_change
	if l0 == 0:
	    l0 = 0.00001
	result = (l0 - l1) + (sample * math.log((l1/l0)))
	return result

    def reset(self):
	self.cusum_log_likelihood = 0.0
	self.min_cusum_log_likelihood = 0.0
	self.sample_sum = 0.0
	self.sample_count = 0
	self.change_point = -1

    def detectChange(self, new_sample):
	# compute current mean
	self.sample_count += 1
	self.sample_sum += new_sample
	current_mean = self.sample_sum/self.sample_count

	#if new mean is negative then skip to next sample 
	if(current_mean + self.change_magnitude < 1):
	    return -1

        #compute the instantenous log likelihood for the new sample
	self.cusum_log_likelihood += self.poisson_log_likelihood(current_mean, current_mean + self.change_magnitude, new_sample)

	if self.cusum_log_likelihood < self.min_cusum_log_likelihood:
	    self.min_cusum_log_likelihood = self.cusum_log_likelihood
	    self.change_point = self.sample_count
        	    
	gk = self.cusum_log_likelihood - self.min_cusum_log_likelihood
	if gk > self.decision_threshold:
	    result = self.change_point
	    #print "Change point: ", self.change_point, "Current mean: ", current_mean, "Change magnitude: ", self.change_magnitude 
	    return result
	#else:
	#print "No change detected, still in the old distribution"
	return -1    	



# q1 detects change of mean to 3
#q1 = Cusum(5,40)

#poisson0 = np.random.poisson(1,25)
#poisson1 = np.random.poisson(2,25)
#poisson2 = np.random.poisson(3,25)
#poisson3 = np.random.poisson(5,100)

#sequence = np.concatenate((poisson0, poisson1, poisson2, poisson3),axis=0)
#sample_list = sequence.tolist()


#change_point1 = -1
#sample_count = 0
#for sample in sample_list:
#    sample_count += 1
#    print sample_count
#    if(change_point1 == -1):
#        change_point1 = q1.detectChange(sample)
    


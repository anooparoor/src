# implement cusum algorithm for online change detection in crowded environments
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import poisson
from scipy.stats import gamma

# bayes_filter that takes a sequence of inputs z(1:t) and returns the p(m|z(1:t)) where m is the parameter one is trying to estimate
# In this case inputs are the crowd counts for single grid cell and crowd arrival rate lambda is tracked 
 
class bayes_filter:
    def __init__(self, prior_alpha, prior_beta, lambda_x):
	# the prior modeled as a gamma distribution
        self.alpha = prior_alpha
	self.beta = prior_beta
	self.x = lambda_x
	self.log_odds = 0.0
	
    # update the filter given a new sample z
    def update(self, z):
	# compute p(x)
	p_x = gamma.pmf(self.x, a=self.alpha, scale=(1/self.beta))
	
	# compute p(x|z), which is the posterior of gamma distribution as MAP estimate
	posterior_alpha = self.alpha + z
	posterior_beta = self.beta + 1
	p_x_given_z = gamma.pmf(self.x, a=posterior_alpha, scale=(1/posterior_beta))

	# update the log odds ratio using the formula
	self.log_odds += math.log(p_x/(1-p_x)) + math.log(p_x_given_z/(1-p_x_given_z))
    
    def expected 

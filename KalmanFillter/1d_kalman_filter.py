
from math import *

def gaussian_f(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5*(x-mu)**2 / sigma2)



def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]


measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.


def calculate():
    res = []
    for i in range(len(measurements)):
        update_ = update(mu,sig,measurements[i],measurement_sig)
        #print(update_)
        res.append(update_)
        predict_ = predict(update_[0],update_[1],motion[i],motion_sig)
        #print(predict_)
        res.append(predict_)
    return res

if __name__== "__main__":
    #print(gaussian_f(10.,4.,10.))
    #print(update(10.,9.,12., 4.))
    print(calculate())

        

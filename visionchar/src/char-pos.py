import re
import matplotlib.pyplot as plt
import numpy as np
data = """
[Sample{leftSpeed=0.0, rightSpeed=0.0, leftEncoder=0.0, rightEncoder=0.0}, Sample{leftSpeed=0.03, rightSpeed=0.03, leftEncoder=0.0, rightEncoder=0.0}, Sample{leftSpeed=0.06, rightSpeed=0.06, leftEncoder=418.0, rightEncoder=352.0}, Sample{leftSpeed=0.09, rightSpeed=0.09, leftEncoder=832.0, rightEncoder=748.0}, Sample{leftSpeed=0.12, rightSpeed=0.12, leftEncoder=1265.0, rightEncoder=1197.0}, Sample{leftSpeed=0.15, rightSpeed=0.15, leftEncoder=1579.0, rightEncoder=1533.0}, Sample{leftSpeed=0.18, rightSpeed=0.18, leftEncoder=1997.0, rightEncoder=1935.0}, Sample{leftSpeed=0.21, rightSpeed=0.21, leftEncoder=2402.0, rightEncoder=2297.0}, Sample{leftSpeed=0.24, rightSpeed=0.24, leftEncoder=2742.0, rightEncoder=2708.0}, Sample{leftSpeed=0.27, rightSpeed=0.27, leftEncoder=3211.0, rightEncoder=3071.0}, Sample{leftSpeed=0.30000000000000004, rightSpeed=0.30000000000000004, leftEncoder=3410.0, rightEncoder=3366.0}, Sample{leftSpeed=0.33000000000000007, rightSpeed=0.33000000000000007, leftEncoder=3906.0, rightEncoder=3837.0}, Sample{leftSpeed=0.3600000000000001, rightSpeed=0.3600000000000001, leftEncoder=4495.0, rightEncoder=4202.0}, Sample{leftSpeed=0.3900000000000001, rightSpeed=0.3900000000000001, leftEncoder=4599.0, rightEncoder=4611.0}, Sample{leftSpeed=0.42000000000000015, rightSpeed=0.42000000000000015, leftEncoder=5038.0, rightEncoder=5004.0}, Sample{leftSpeed=0.4500000000000002, rightSpeed=0.4500000000000002, leftEncoder=5411.0, rightEncoder=5364.0}, Sample{leftSpeed=0.4800000000000002, rightSpeed=0.4800000000000002, leftEncoder=5835.0, rightEncoder=5600.0}, Sample{leftSpeed=0.5100000000000002, rightSpeed=0.5100000000000002, leftEncoder=6279.0, rightEncoder=6212.0}, Sample{leftSpeed=0.5400000000000003, rightSpeed=0.5400000000000003, leftEncoder=6579.0, rightEncoder=6547.0}, Sample{leftSpeed=0.5700000000000003, rightSpeed=0.5700000000000003, leftEncoder=6986.0, rightEncoder=6690.0}, Sample{leftSpeed=0.6000000000000003, rightSpeed=0.6000000000000003, leftEncoder=7377.0, rightEncoder=6960.0}, Sample{leftSpeed=0.6300000000000003, rightSpeed=0.6300000000000003, leftEncoder=7449.0, rightEncoder=7095.0}, Sample{leftSpeed=0.6600000000000004, rightSpeed=0.6600000000000004, leftEncoder=8458.0, rightEncoder=7675.0}, Sample{leftSpeed=0.6900000000000004, rightSpeed=0.6900000000000004, leftEncoder=8399.0, rightEncoder=8240.0}, Sample{leftSpeed=0.7200000000000004, rightSpeed=0.7200000000000004, leftEncoder=8549.0, rightEncoder=8197.0}, Sample{leftSpeed=0.7500000000000004, rightSpeed=0.7500000000000004, leftEncoder=8944.0, rightEncoder=8046.0}, Sample{leftSpeed=0.7800000000000005, rightSpeed=0.7800000000000005, leftEncoder=9147.0, rightEncoder=8329.0}, Sample{leftSpeed=0.8100000000000005, rightSpeed=0.8100000000000005, leftEncoder=8850.0, rightEncoder=8248.0}, Sample{leftSpeed=0.8400000000000005, rightSpeed=0.8400000000000005, leftEncoder=8829.0, rightEncoder=8332.0}, Sample{leftSpeed=0.8700000000000006, rightSpeed=0.8700000000000006, leftEncoder=8849.0, rightEncoder=8302.0}, Sample{leftSpeed=0.9000000000000006, rightSpeed=0.9000000000000006, leftEncoder=8725.0, rightEncoder=8262.0}, Sample{leftSpeed=0.9300000000000006, rightSpeed=0.9300000000000006, leftEncoder=8612.0, rightEncoder=8291.0}, Sample{leftSpeed=0.9600000000000006, rightSpeed=0.9600000000000006, leftEncoder=8705.0, rightEncoder=8322.0}, Sample{leftSpeed=0.9900000000000007, rightSpeed=0.9900000000000007, leftEncoder=8707.0, rightEncoder=8507.0}]
"""
leftSpeeds = np.array([float(x) for x in re.findall(r"leftSpeed=([\w.-]+)", data)])
rightSpeeds = np.array([float(x) for x in re.findall(r"rightSpeed=([\w.-]+)", data)])
leftEncoders = np.array([float(x) for x in re.findall(r"leftEncoder=([\w.-]+)", data)])
rightEncoders = np.array([float(x) for x in re.findall(r"rightEncoder=([\w.-]+)", data)])

plt.scatter(leftSpeeds,leftEncoders)
plt.scatter(rightSpeeds,rightEncoders)
plt.show()

'''
positive = turns>=0
negative = turns<=0
positive_m, positive_b = np.polyfit(ticks[positive],deltaDegs[positive],1)
negative_m, negative_b = np.polyfit(ticks[negative],deltaDegs[negative],1)

def plt_mxb(m,b):
    data = np.array([np.amin(ticks),np.amax(ticks)])
    plt.plot(data, data*m+b, '--')

print(positive_m,positive_b,negative_m,negative_b)

plt.scatter(ticks,deltaDegs)
plt_mxb(positive_m, positive_b)
plt_mxb(negative_m, negative_b)
plt.show()
'''
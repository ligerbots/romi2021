import re
import matplotlib.pyplot as plt
import numpy as np
data = """
[Sample{leftSpeed=0.8700000000000006, rightSpeed=0.8700000000000006, leftEncoder=8473.0, rightEncoder=8409.0}, Sample{leftSpeed=0.3600000000000001, rightSpeed=0.3600000000000001, leftEncoder=4182.0, rightEncoder=4168.0}, Sample{leftSpeed=0.4500000000000002, rightSpeed=0.4500000000000002, leftEncoder=5236.0, rightEncoder=5217.0}, Sample{leftSpeed=0.6900000000000004, rightSpeed=0.6900000000000004, leftEncoder=7863.0, rightEncoder=7541.0}, Sample{leftSpeed=0.5100000000000002, rightSpeed=0.5100000000000002, leftEncoder=5926.0, rightEncoder=5649.0}, Sample{leftSpeed=0.6300000000000003, rightSpeed=0.6300000000000003, leftEncoder=7260.0, rightEncoder=6876.0}, Sample{leftSpeed=0.06, rightSpeed=0.06, leftEncoder=385.0, rightEncoder=299.0}, Sample{leftSpeed=0.9300000000000006, rightSpeed=0.9300000000000006, leftEncoder=8250.0, rightEncoder=7923.0}, Sample{leftSpeed=0.18, rightSpeed=0.18, leftEncoder=1930.0, rightEncoder=1816.0}, Sample{leftSpeed=0.8100000000000005, rightSpeed=0.8100000000000005, leftEncoder=8487.0, rightEncoder=8223.0}, Sample{leftSpeed=0.12, rightSpeed=0.12, leftEncoder=1211.0, rightEncoder=1078.0}, Sample{leftSpeed=0.8400000000000005, rightSpeed=0.8400000000000005, leftEncoder=8534.0, rightEncoder=8251.0}, Sample{leftSpeed=0.7500000000000004, rightSpeed=0.7500000000000004, leftEncoder=8524.0, rightEncoder=8163.0}, Sample{leftSpeed=0.24, rightSpeed=0.24, leftEncoder=2719.0, rightEncoder=2561.0}, Sample{leftSpeed=0.33000000000000007, rightSpeed=0.33000000000000007, leftEncoder=3778.0, rightEncoder=3631.0}, Sample{leftSpeed=0.03, rightSpeed=0.03, leftEncoder=0.0, rightEncoder=0.0}, Sample{leftSpeed=0.5700000000000003, rightSpeed=0.5700000000000003, leftEncoder=6656.0, rightEncoder=6392.0}, Sample{leftSpeed=0.7800000000000005, rightSpeed=0.7800000000000005, leftEncoder=8533.0, rightEncoder=8486.0}, Sample{leftSpeed=0.30000000000000004, rightSpeed=0.30000000000000004, leftEncoder=3451.0, rightEncoder=3261.0}, Sample{leftSpeed=0.6600000000000004, rightSpeed=0.6600000000000004, leftEncoder=7615.0, rightEncoder=7270.0}, Sample{leftSpeed=0.5400000000000003, rightSpeed=0.5400000000000003, leftEncoder=6365.0, rightEncoder=6069.0}, Sample{leftSpeed=0.7200000000000004, rightSpeed=0.7200000000000004, leftEncoder=8359.0, rightEncoder=8023.0}, Sample{leftSpeed=0.21, rightSpeed=0.21, leftEncoder=2219.0, rightEncoder=2081.0}, Sample{leftSpeed=0.3900000000000001, rightSpeed=0.3900000000000001, leftEncoder=4471.0, rightEncoder=4412.0}, Sample{leftSpeed=0.09, rightSpeed=0.09, leftEncoder=764.0, rightEncoder=636.0}, Sample{leftSpeed=0.42000000000000015, rightSpeed=0.42000000000000015, leftEncoder=4861.0, rightEncoder=4732.0}, Sample{leftSpeed=0.9600000000000006, rightSpeed=0.9600000000000006, leftEncoder=8521.0, rightEncoder=8199.0}, Sample{leftSpeed=0.9900000000000007, rightSpeed=0.9900000000000007, leftEncoder=8821.0, rightEncoder=8176.0}, Sample{leftSpeed=0.4800000000000002, rightSpeed=0.4800000000000002, leftEncoder=5564.0, rightEncoder=5439.0}, Sample{leftSpeed=0.15, rightSpeed=0.15, leftEncoder=1472.0, rightEncoder=1379.0}, Sample{leftSpeed=0.27, rightSpeed=0.27, leftEncoder=2927.0, rightEncoder=2787.0}, Sample{leftSpeed=0.0, rightSpeed=0.0, leftEncoder=0.0, rightEncoder=0.0}, Sample{leftSpeed=0.9000000000000006, rightSpeed=0.9000000000000006, leftEncoder=8461.0, rightEncoder=8147.0}, Sample{leftSpeed=0.6000000000000003, rightSpeed=0.6000000000000003, leftEncoder=6832.0, rightEncoder=6478.0}]
"""
data = """
[Sample{leftSpeed=0.18, rightSpeed=0.18, leftEncoder=1674.0, rightEncoder=1668.0}, Sample{leftSpeed=0.09, rightSpeed=0.09, leftEncoder=667.0, rightEncoder=628.0}, Sample{leftSpeed=0.5700000000000003, rightSpeed=0.5700000000000003, leftEncoder=6516.0, rightEncoder=6357.0}, Sample{leftSpeed=0.0, rightSpeed=0.0, leftEncoder=0.0, rightEncoder=0.0}, Sample{leftSpeed=0.12, rightSpeed=0.12, leftEncoder=1018.0, rightEncoder=924.0}, Sample{leftSpeed=0.8400000000000005, rightSpeed=0.8400000000000005, leftEncoder=8490.0, rightEncoder=8093.0}, Sample{leftSpeed=0.4500000000000002, rightSpeed=0.4500000000000002, leftEncoder=4939.0, rightEncoder=5001.0}, Sample{leftSpeed=0.5100000000000002, rightSpeed=0.5100000000000002, leftEncoder=5889.0, rightEncoder=5640.0}, Sample{leftSpeed=0.6000000000000003, rightSpeed=0.6000000000000003, leftEncoder=6827.0, rightEncoder=6504.0}, Sample{leftSpeed=0.8700000000000006, rightSpeed=0.8700000000000006, leftEncoder=8876.0, rightEncoder=8160.0}, Sample{leftSpeed=0.4800000000000002, rightSpeed=0.4800000000000002, leftEncoder=5557.0, rightEncoder=5463.0}, Sample{leftSpeed=0.27, rightSpeed=0.27, leftEncoder=2942.0, rightEncoder=3108.0}, Sample{leftSpeed=0.6600000000000004, rightSpeed=0.6600000000000004, leftEncoder=7608.0, rightEncoder=7308.0}, Sample{leftSpeed=0.21, rightSpeed=0.21, leftEncoder=2186.0, rightEncoder=2068.0}, Sample{leftSpeed=0.33000000000000007, rightSpeed=0.33000000000000007, leftEncoder=3643.0, rightEncoder=3573.0}, Sample{leftSpeed=0.03, rightSpeed=0.03, leftEncoder=0.0, rightEncoder=0.0}, Sample{leftSpeed=0.3900000000000001, rightSpeed=0.3900000000000001, leftEncoder=4340.0, rightEncoder=4317.0}, Sample{leftSpeed=0.9300000000000006, rightSpeed=0.9300000000000006, leftEncoder=8600.0, rightEncoder=8189.0}, Sample{leftSpeed=0.7200000000000004, rightSpeed=0.7200000000000004, leftEncoder=8358.0, rightEncoder=8014.0}, Sample{leftSpeed=0.5400000000000003, rightSpeed=0.5400000000000003, leftEncoder=6245.0, rightEncoder=6175.0}, Sample{leftSpeed=0.6300000000000003, rightSpeed=0.6300000000000003, leftEncoder=7154.0, rightEncoder=6835.0}, Sample{leftSpeed=0.7800000000000005, rightSpeed=0.7800000000000005, leftEncoder=8579.0, rightEncoder=8177.0}, Sample{leftSpeed=0.15, rightSpeed=0.15, leftEncoder=1471.0, rightEncoder=1380.0}, Sample{leftSpeed=0.06, rightSpeed=0.06, leftEncoder=354.0, rightEncoder=283.0}, Sample{leftSpeed=0.24, rightSpeed=0.24, leftEncoder=2564.0, rightEncoder=2479.0}, Sample{leftSpeed=0.30000000000000004, rightSpeed=0.30000000000000004, leftEncoder=3306.0, rightEncoder=3423.0}, Sample{leftSpeed=0.8100000000000005, rightSpeed=0.8100000000000005, leftEncoder=8734.0, rightEncoder=8216.0}, Sample{leftSpeed=0.7500000000000004, rightSpeed=0.7500000000000004, leftEncoder=8749.0, rightEncoder=8219.0}, Sample{leftSpeed=0.6900000000000004, rightSpeed=0.6900000000000004, leftEncoder=8099.0, rightEncoder=7538.0}, Sample{leftSpeed=0.9600000000000006, rightSpeed=0.9600000000000006, leftEncoder=8706.0, rightEncoder=8208.0}, Sample{leftSpeed=0.42000000000000015, rightSpeed=0.42000000000000015, leftEncoder=4934.0, rightEncoder=4730.0}, Sample{leftSpeed=0.3600000000000001, rightSpeed=0.3600000000000001, leftEncoder=4177.0, rightEncoder=4017.0}, Sample{leftSpeed=0.9000000000000006, rightSpeed=0.9000000000000006, leftEncoder=8706.0, rightEncoder=8224.0}, Sample{leftSpeed=0.9900000000000007, rightSpeed=0.9900000000000007, leftEncoder=8781.0, rightEncoder=8243.0}]
"""
leftSpeeds = np.array([float(x) for x in re.findall(r"leftSpeed=([\w.-]+)", data)])
rightSpeeds = np.array([float(x) for x in re.findall(r"rightSpeed=([\w.-]+)", data)])
leftEncoders = np.array([float(x) for x in re.findall(r"leftEncoder=([\w.-]+)", data)])
rightEncoders = np.array([float(x) for x in re.findall(r"rightEncoder=([\w.-]+)", data)])

plt.scatter(leftSpeeds,leftEncoders, color="red")
plt.scatter(rightSpeeds,rightEncoders, color="blue")
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
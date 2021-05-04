import math
import re
import matplotlib.pyplot as plt
import numpy as np
data = """
[Sample{movementx=1.0309777815188765, movementy=-0.3266380555049326, movementdeg=-27.32003784179687, ticks=96, speed=0.8}, Sample{movementx=0.737578060501124, movementy=-0.1679213431031179, movementdeg=-18.10575866699219, ticks=66, speed=0.8}, Sample{movementx=0.09395691541411827, movementy=-0.023859749604992722, movementdeg=-1.6193847656249984, ticks=14, speed=0.8}, Sample{movementx=0.9984626853401843, movementy=-0.36081642693659094, movementdeg=-32.32231140136719, ticks=94, speed=0.8}, Sample{movementx=0.9120607763427031, movementy=-0.2851398425392243, movementdeg=-29.15615844726562, ticks=84, speed=0.8}, Sample{movementx=0.3374320892247752, movementy=-0.04887380566043903, movementdeg=-5.609054565429685, ticks=34, speed=0.8}, Sample{movementx=0.3957734955376765, movementy=-0.08134767932184733, movementdeg=-15.262420654296871, ticks=40, speed=0.8}, Sample{movementx=-7.409277530463799E-4, movementy=-9.932570753018505E-4, movementdeg=0.00889587402343802, ticks=0, speed=0.8}, Sample{movementx=0.4887501619904519, movementy=-0.12369401641496772, movementdeg=-17.399261474609375, ticks=48, speed=0.8}, Sample{movementx=0.7872112780797016, movementy=-0.19667369663669493, movementdeg=-21.446533203125004, ticks=72, speed=0.8}, Sample{movementx=0.533490456863705, movementy=-0.16017754552220653, movementdeg=-20.424926757812504, ticks=50, speed=0.8}, Sample{movementx=0.4725908885151722, movementy=-0.07405272053372222, movementdeg=-13.063980102539066, ticks=44, speed=0.8}, Sample{movementx=0.162708524758643, movementy=-0.025853071109907658, movementdeg=0.9913482666015679, ticks=20, speed=0.8}, Sample{movementx=3.074495361046547E-5, movementy=9.245124471922204E-4, movementdeg=-0.025161743164061792, ticks=2, speed=0.8}, Sample{movementx=0.1818991679780101, movementy=-0.036133192876657214, movementdeg=-3.0850830078124956, ticks=24, speed=0.8}, Sample{movementx=0.6436715131637541, movementy=-0.19101860036408627, movementdeg=-26.86129760742187, ticks=64, speed=0.8}, Sample{movementx=0.361955986318795, movementy=-0.055364756349204125, movementdeg=-9.202224731445316, ticks=36, speed=0.8}, Sample{movementx=0.4188731747553883, movementy=-0.07356843190743567, movementdeg=-12.368927001953132, ticks=42, speed=0.8}, Sample{movementx=0.5932418829848796, movementy=-0.12818331732523372, movementdeg=-16.057632446289052, ticks=56, speed=0.8}, Sample{movementx=0.10187095743517383, movementy=-0.02072272584790958, movementdeg=-0.8872680664062464, ticks=16, speed=0.8}, Sample{movementx=0.18470665111650825, movementy=-0.02702388319291922, movementdeg=-2.2312469482421866, ticks=22, speed=0.8}, Sample{movementx=0.025843670209823914, movementy=-0.011091769735278928, movementdeg=-2.340423583984373, ticks=10, speed=0.8}, Sample{movementx=0.27701115655370756, movementy=-0.044232028654363056, movementdeg=-6.799682617187499, ticks=28, speed=0.8}, Sample{movementx=0.38726259454550094, movementy=-0.07169419409043143, movementdeg=-10.151885986328127, ticks=38, speed=0.8}, Sample{movementx=0.9000972912376597, movementy=-0.40145835156135334, movementdeg=-39.743927001953146, ticks=92, speed=0.8}, Sample{movementx=0.5480704433239947, movementy=-0.13652317038992637, movementdeg=-20.02742004394531, ticks=54, speed=0.8}, Sample{movementx=0.14912156531504595, movementy=-0.02098816557898254, movementdeg=-1.9257659912109424, ticks=18, speed=0.8}, Sample{movementx=0.8009078989405185, movementy=-0.34596921983987355, movementdeg=-36.50906372070313, ticks=82, speed=0.8}, Sample{movementx=0.8242507862593682, movementy=-0.2629865326560153, movementdeg=-29.975158691406243, ticks=78, speed=0.8}, Sample{movementx=0.28467167816814654, movementy=-0.04347875421983442, movementdeg=-6.462692260742189, ticks=30, speed=0.8}, Sample{movementx=0.8654396818939992, movementy=-0.32049215760083294, movementdeg=-35.72607421875, ticks=88, speed=0.8}, Sample{movementx=0.8193995255768107, movementy=-0.264726843616832, movementdeg=-30.60275268554687, ticks=80, speed=0.8}, Sample{movementx=0.9066120141053007, movementy=-0.28970437430614915, movementdeg=-33.748611450195305, ticks=90, speed=0.8}, Sample{movementx=0.006676379411312267, movementy=-0.005111427653088195, movementdeg=1.8604583740234384, ticks=6, speed=0.8}, Sample{movementx=0.6432722987784256, movementy=-0.14423896083957916, movementdeg=-18.0538330078125, ticks=62, speed=0.8}, Sample{movementx=0.9003320098994534, movementy=-0.24118353721665736, movementdeg=-29.07893371582031, ticks=86, speed=0.8}, Sample{movementx=0.023496846846158357, movementy=-0.0085882336680139, movementdeg=-0.024963378906250454, ticks=8, speed=0.8}, Sample{movementx=0.2968877601689529, movementy=-0.05629527908327796, movementdeg=-8.81739807128907, ticks=32, speed=0.8}, Sample{movementx=0.7604363010073704, movementy=-0.2259017511513205, movementdeg=-27.87825012207031, ticks=76, speed=0.8}, Sample{movementx=0.7136243626099398, movementy=-0.14791922613886582, movementdeg=-26.296722412109375, ticks=70, speed=0.8}, Sample{movementx=0.08764028036687854, movementy=-0.0027831249062221855, movementdeg=-0.5895690917968744, ticks=12, speed=0.8}, Sample{movementx=0.5842029681220019, movementy=-0.10583545749479872, movementdeg=-14.52876281738281, ticks=58, speed=0.8}, Sample{movementx=0.5144620755631037, movementy=-0.08881503144902503, movementdeg=-16.558380126953125, ticks=52, speed=0.8}, Sample{movementx=0.5933723931670641, movementy=-0.1620369927137427, movementdeg=-20.538528442382812, ticks=60, speed=0.8}, Sample{movementx=0.8004585877029466, movementy=-0.19771474409980427, movementdeg=-27.477157592773445, ticks=74, speed=0.8}, Sample{movementx=0.4749665272655825, movementy=-0.10333756123449173, movementdeg=-13.629638671874995, ticks=46, speed=0.8}, Sample{movementx=0.9478085534159936, movementy=-0.4183231186572347, movementdeg=-41.39837646484374, ticks=98, speed=0.8}, Sample{movementx=0.6952424039280269, movementy=-0.156021259002472, movementdeg=-25.203323364257812, ticks=68, speed=0.8}, Sample{movementx=0.003696972966687302, movementy=-0.0010852560999009588, movementdeg=-0.5090484619140632, ticks=4, speed=0.8}, Sample{movementx=0.2339986623954282, movementy=-0.02597492430337557, movementdeg=-2.5012664794921866, ticks=26, speed=0.8}]
"""

movementx = np.array([float(x) for x in re.findall(r"movementx=([\w.-]+)", data)])
movementy = np.array([float(x) for x in re.findall(r"movementy=([\w.-]+)", data)])
movementdeg = np.array([float(x) for x in re.findall(r"movementdeg=([\w.-]+)", data)])
ticks = np.array([float(x) for x in re.findall(r"ticks=([\w.-]+)", data)])
speed = np.array([float(x) for x in re.findall(r"speed=([\w.-]+)", data)])

plt.scatter(movementx,movementy, color="red")
for i in range(len(movementx)):
    plt.annotate(str(ticks[i]),(movementx[i],movementy[i]))
    plt.arrow(movementx[i],movementy[i],math.cos(math.radians(movementdeg[i]))*.03,math.sin(math.radians(movementdeg[i]))*.03)

movementx_2, movementx_1, movementx_0 = np.polyfit(ticks,movementx,2)
movementy_2, movementy_1, movementy_0 = np.polyfit(ticks,movementy,2)

data = np.arange(np.amin(ticks)+20,np.amax(ticks),.1)
xs = (data**2)*movementx_2+data*movementx_1+movementx_0
ys = (data**2)*movementy_2+data*movementy_1+movementy_0

plt.plot(xs, ys, '--')

distance_s = []
angle_s = []
for i in range(len(xs)):
    distance_s.append(math.sqrt(xs[i]**2+ys[i]**2))
    angle_s.append(math.atan2(ys[i],xs[i]))


distance_2, distance_1, distance_0 = np.polyfit(distance_s,data,2)
angle_2, angle_1, angle_0 = np.polyfit(data,angle_s,2)


def plte():
    dists = np.arange(np.amin(distance_s),np.amax(distance_s),.1)
    ticks = (dists**2)*distance_2 + dists*distance_1 + distance_0
    angs = (ticks**2)*angle_2 + ticks*angle_1 + angle_0
    xs = np.cos(angs)*dists
    ys = np.sin(angs)*dists
    plt.plot(xs, ys, '-')
plte()

print(distance_2, distance_1, distance_0,angle_2, angle_1, angle_0)
plt.show()



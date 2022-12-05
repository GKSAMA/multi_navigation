import os
import math
import pandas as pd
import numpy as np
import sys
import matplotlib.pyplot as plt
from copy import deepcopy

waypointspath = "/home/gk/Documents/multi_turtlebot3_navigation/src/multi_turtlebot3_navigation/config/waypoints.data"
datapath = "/home/gk/Documents/DataRecord"
distdatafile = "minobsdist.data"
pose2ddatafile = "pose2Ddata.data"
veldatafile = "veldata.data"
segtimedatafile = "segtime.data"
globalpathdistfile = "dist2goal.data"


def load_pathdist_data(datafile):
    datas = []
    labels = []
    with open(os.path.join(datapath,datafile), 'r') as f:
        for line in f.readlines():
            line_data = line.split(' ')
            line_data[0] = float(line_data[0])
            line_data[1] = float(line_data[1])
            line_data[2] = int(line_data[2])
            line_data[3] = int(line_data[3])
            datas.append(line_data)
    datalist = []
    start_time = datas[0][0]
    datatemp = []
    idx = 0
    labels.append(idx)
    for data in datas:
        if(data[3] != idx):
            datalist.append(deepcopy(datatemp))
            idx = data[3]
            labels.append(idx)
            datatemp = []
        data[0] = data[0] - start_time;
        datatemp.append([data[0], data[1], data[2]])
    datalist.append(datatemp)
    return datalist,labels

# data format : [time, x, y, theta]
def load_waypoints_data(datafile):
    datas = []
    with open(datafile, 'r') as f:
        for line in f.readlines():
            line_data = line.split(' ')
            line_data[0] = float(line_data[0])
            line_data[1] = float(line_data[1])
            line_data[2] = float(line_data[2])
            line_data[3] = float(line_data[3])
            datas.append(line_data)
    return datas

# data format : ["time" "data" "idx"]
def load_data(datafile):
    datas = []
    labels = []
    with open(os.path.join(datapath,datafile), 'r') as f:
        for line in f.readlines():
            line_data = line.split(' ')
            line_data[2] = int(line_data[2][0])
            line_data[0] = float(line_data[0])
            line_data[1] = float(line_data[1])
            datas.append(line_data)
    datalist = []
    start_time = datas[0][0]
    datatemp = []
    idx = 0
    labels.append(idx)
    for data in datas:
        if data[2] != idx:
            datalist.append(deepcopy(datatemp))
            idx = data[2]
            labels.append(idx)
            datatemp = []
        data[0] = data[0]-start_time
        datatemp.append([data[0],data[1]])
    datalist.append(datatemp)
    return datalist,labels

# data format : [time, waypoint, idx]
def load_segtime_data(datafile):
    datas = []
    labels = []
    with open(os.path.join(datapath,datafile)) as f:
        for line in f.readlines():
            line_data  = line.split(" ")
            line_data[0] = float(line_data[0])
            line_data[1] = int(line_data[1])
            line_data[2] = int(line_data[2])
            datas.append(line_data)
    datalist=[]
    datatmp = []
    idx = 0
    labels.append(idx)
    for data in datas:
        if data[2] != idx:
            datalist.append(deepcopy(datatmp))
            idx = data[2]
            labels.append(idx)
            datatmp = []
        datatmp.append(data)
    datalist.append(datatmp)
    return datalist,labels

# data format: ["time" "data1" "data2" "data3" "idx"] 
def load_xyz_data(datafile):
    datas = []
    labels = []
    with open(os.path.join(datapath,datafile),'r') as f:
        for line in f.readlines():
            line_data = line.split(' ')
            line_data[0] = float(line_data[0])
            line_data[1] = float(line_data[1])
            line_data[2] = float(line_data[2])
            line_data[3] = float(line_data[3])
            line_data[4] = int(line_data[4])
            line_data[5] = int(line_data[5][0])
            datas.append(line_data)
    datalist = []
    start_time = datas[0][0]
    datatmp = []
    idx = 0
    labels.append(idx)
    for data in datas:
        if data[5] != idx:
            datalist.append(deepcopy(datatmp))
            idx = data[5]
            labels.append(idx)
            datatmp = []
        data[0] = data[0]-start_time
        data = np.array(data)
        datatmp.append(data[:-1])
    datalist.append(datatmp)
    # print(datalist)
    return datalist,labels
# make datas' time start from 0 in their runtime and fill data to other short data
def dataTimeProcess(datalist):
    for i in range(len(datalist)-1,0,-1):
        for j in range(len(datalist[i])-1,-1,-1):
            datalist[i][j][0] = datalist[i][j][0]-datalist[i][0][0]
    return datalist

def dataFill(datalist):
    maxdatacnt = 0
    for i in range(len(datalist)):
        curcnts = len(datalist[i])
        maxdatacnt = max(maxdatacnt,curcnts)
    for i in range(len(datalist)):
        if(len(datalist[i])<maxdatacnt):
            for j in range(len(datalist[i]),maxdatacnt):
                datatmp = []
                datatmp.append(datalist[i][j-1][0]+0.1)
                for k in range(1,len(datalist[i][j-1])):
                    datatmp.append(datalist[i][j-1][k])
                datalist[i].append(deepcopy(datatmp))
                datatmp = []
    return datalist,maxdatacnt

def wpPathDataFill(datalist,wpnum):
    datas = []
    maxcountlist = []
    robots = len(datalist)
    for wp in range(1,wpnum):
        wplist = []
        for robot in range(robots):
            robotdatalist = []
            for distdata in datalist[robot]:
                if distdata[2] == wp:
                    robotdatalist.append(distdata[1])
                elif distdata[2] > wp:
                    break
            wplist.append(deepcopy(robotdatalist))
        maxlen = 0;
        for p in wplist:
            maxlen = max(maxlen, len(p))
        maxcountlist.append(maxlen)
        # datafill
        for robotdata in wplist:
            curlen = len(robotdata)
            for j in range(curlen, maxlen):
                robotdata.append(robotdata[curlen - 1])
        datas.append(deepcopy(wplist))
    return datas,maxcountlist


def dist2ObsDrawLine(datas, labels,maxdatacounts):
    x = []
    x = list(range(maxdatacounts))
    y = []
    for data in datas:
        data = np.array(data)
        # print(data)
        y.append(data[:,1])
    plt.figure(figsize=(20,10),dpi=70)
    plt.plot(x,y[0],c='blue',label='robot'+str(labels[0]))
    plt.plot(x,y[1],c='green',label='robot'+str(labels[1]))
    plt.plot(x,y[2],c='red',label='robot'+str(labels[2]))
    plt.plot(x,y[3],c='orange',label='robot'+str(labels[3]))
    plt.legend(loc='best')
    plt.xlabel('Time/s')
    plt.ylabel('Dist/m')
    plt.title('Distance to the nearest obstacle',fontdict={'size':20})
    plt.show()

def poseDrawLine(datas, maxdatacounts):
    x = list(range(maxdatacounts))
    y = []
    datax = []
    datay = []
    datath = []
    robotdata = []
    for i in range(len(datas)):
        for j in range(len(datas[i])):
            datax.append(datas[i][j][1])
            datay.append(datas[i][j][2])
            datath.append(datas[i][j][3])
        robotdata.append(deepcopy(datax))
        robotdata.append(deepcopy(datay))
        robotdata.append(deepcopy(datath))
        datax = []
        datay = []
        datath = []
        y.append(deepcopy(robotdata))
        robotdata = []
    fig,axs = plt.subplots(nrows = len(y[0]), ncols = 1,figsize=(20,6),dpi = 70)
    labels = ['eband','dwa','teb','dwb']
    colors = ['blue','green','red','orange']
    datalabels = ['x','y','theta']
    for i in range(len(y[0])):
        for j in range(len(y)):
            axs[i].plot(x,y[j][i],c=colors[j],label=str(labels[j])+datalabels[i])
        axs[i].legend(loc='best')
        axs[i].set_xlabel('Time/s',fontdict={'size':16})
        ylabel = ''
        if datalabels[i]=='x':
            ylabel = 'x/m'
        elif datalabels[i]=='y':
            ylabel = 'y/m'
        else:
            ylabel = 'theta/rad'
        axs[i].set_ylabel(ylabel,fontdict={'size':16})
    fig.autofmt_xdate()
    plt.title('Pose Information',fontdict={'size':20})
    plt.show()

def totlalDistDraw(datas,labels,maxdatacounts):
    sumdist = 0.0
    waypoint = 1
    x = list(range(maxdatacounts))
    distance = []
    y = []
    for i in range(len(datas)):
        for j in range(1,len(datas[i])):
            if(waypoint!=datas[i][j][4]):
                distance.append(deepcopy(sumdist))
                sumdist = 0
                waypoint = datas[i][j][4]
            sumdist += math.sqrt(math.pow(datas[i][j][1]-datas[i][j-1][1],2)+
                                math.pow(datas[i][j][2]-datas[i][j-1][2],2));
        distance.append(deepcopy(sumdist))
        y.append(deepcopy(distance))
        distance = []
        sumdist = 0
        waypoint = 1
    # print(y)
    bar_width = 0.25
    plt.bar(x=x,height=y[0],label='eband',color='blue',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+bar_width,height=y[1],label="dwa",color='green',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+2*bar_width,height=y[2],label="teb",color='red',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+3*bar_width,height=y[3],label="dwb",color='orange',alpha=0.8,width = bar_width)
    
    for x1,yy in enumerate(y[0]):
        plt.text(x1,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[1]):
        plt.text(x1+bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[2]):
        plt.text(x1+2*bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[3]):
        plt.text(x1+3*bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    plt.xlabel('waypoint')
    plt.ylabel('distance/m')
    plt.legend(loc='best')
    plt.title('Every Segment Distance Cost',fontdict={'size':20})
    plt.show()
    pass

def velDrawLine(datas,maxdatacounts):
    x = list(range(maxdatacounts))
    y = []
    datax = []
    datath = []
    robotdata = []
    for i in range(len(datas)):
        for j in range(len(datas[i])):
            datax.append(datas[i][j][1])
            datath.append(datas[i][j][3])
        robotdata.append(deepcopy(datax))
        robotdata.append(deepcopy(datath))
        datax = []
        datath = []
        y.append(deepcopy(robotdata))
        robotdata = []
    fig,axs = plt.subplots(nrows = len(y[0]), ncols = 1,figsize=(20,6),dpi = 70)
    labels = ['eband','dwa','teb','dwb']
    colors = ['blue','green','red','orange']
    datalabels = ['x','theta']
    for i in range(len(y[0])):
        for j in range(len(y)):
            axs[i].plot(x,y[j][i],c=colors[j],label=str(labels[j])+datalabels[i])
        axs[i].legend(loc='best')
        axs[i].set_xlabel('Time/s',fontdict={'size':16})
        axs[i].set_ylabel('linear/ m/s' if datalabels[i]=='x' else 'angular/ rad/s',fontdict={'size':16})
    fig.autofmt_xdate()
    plt.title('Velocity Information',fontdict={'size':20})
    plt.show()

def accDrawLine(datas,labels,maxdatacounts):
    x = list(range(maxdatacounts)-1)
    y = []
    acctemp = []
    for i in range(len(datas)):
        for j in range(1,len(datas[i])):
            velxdiff = datas[i][j][1] - datas[i][j-1][1]
            velthdiff = datas[i][j][3] - datas[i][j-1][3]
            timediff = datas[i][j][0] - datas[i][j-1][0]
            accx = velxdiff/timediff
            acctemp.append(accx)
        y.append(acctemp)
        acctemp = []
    plt.figure(figsize=(20,10),dpi=70)
    plt.plot(x,y[0],c='blue',label='eband')
    plt.plot(x,y[1],c='green',label='dwa')
    plt.plot(x,y[2],c='red',label='teb')
    plt.plot(x,y[3],c='orange',label='dwb')
    plt.legend(loc='best')
    plt.xlabel('Time/s')
    plt.ylabel('Acc/(m^2/s)')
    plt.title("Accleration Information",fontdict={'size':20})
    plt.show()

def segTimeDrawLine(datas,labels,maxdatacounts):
    x = list(range(maxdatacounts))
    y = []
    for data in datas:
        data = np.array(data)
        y.append(data[:,0])
    bar_width = 0.25
    plt.bar(x=x,height=y[0],label='eband',color='blue',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+bar_width,height=y[1],label="dwa",color='green',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+2*bar_width,height=y[2],label="teb",color='red',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+3*bar_width,height=y[3],label="dwb",color='orange',alpha=0.8,width = bar_width)
    for x1,yy in enumerate(y[0]):
        plt.text(x1,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[1]):
        plt.text(x1+bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[2]):
        plt.text(x1+2*bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[3]):
        plt.text(x1+3*bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    plt.xlabel('waypoint')
    plt.ylabel('time/s')
    plt.legend(loc='best')
    plt.title('Every Segment Time Cost',fontdict={'size':20})
    plt.show()

def distanceToGoalDrawLine(waypointsdatas,posedatas,labels,maxdatacounts):
    x = list(range(maxdatacounts))
    y = []
    distlist = []
    print(len(posedatas[0]),len(posedatas[1]),len(posedatas[2]),len(posedatas[3]))
    for i in range(len(posedatas)):
        home = [posedatas[i][0][1],posedatas[i][0][2]]
        for j in range(len(posedatas[i])):
            pose = [posedatas[i][j][1], posedatas[i][j][2], 0]
            # orientation = [0, 0, data[3], data[4]]
            # yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]), 1 - 2*(orientation[2] * orientation[2] + orientation[1] * orientation[1]))
            wpidx = (int)(posedatas[i][j][4]-1)
            # print(wpidx)
            if(wpidx != len(waypointsdatas)):
                dist = math.sqrt((pose[0]-waypointsdatas[wpidx][0])*(pose[0]-waypointsdatas[wpidx][0]) + (pose[1]-waypointsdatas[wpidx][1])*(pose[1]-waypointsdatas[wpidx][1]))
            else:
                dist = math.sqrt((pose[0]-home[0])*(pose[0]-home[0]) + (pose[1]-home[1])*(pose[1] - home[1]))
            distlist.append(dist)
        y.append(distlist)
        distlist = []
    plt.figure(figsize=(20,10),dpi=70)
    plt.plot(x,y[0],c='blue',label="eband")
    plt.plot(x,y[1],c='green',label='dwa')
    plt.plot(x,y[2],c='red',label='teb')
    plt.plot(x,y[3],c='orange',label='dwb')
    plt.legend(loc='best')
    plt.xlabel("Time/s")
    plt.ylabel("Dist2Goal/m")
    plt.title("Distance to next waypoint",fontdict={'size':20})
    plt.show()
        

def globalDistToGoalDrawLine(globaldistdatas,labels,maxdatacounts):
    x = []
    for maxdatacount in maxdatacounts:
        x.append(list(range(maxdatacount)))
    y = globaldistdatas
    # distlist = []
    # for i in range(len(globaldistdatas)):
    #     for j in range(len(globaldistdatas[i])):
    #         distlist.append(globaldistdatas[i][j][1])
    #     y.append(deepcopy(distlist))
    #     distlist = []
    fig,axs = plt.subplots(nrows = int((len(y) + 1)/2), ncols = 2,figsize=(20,6),dpi = 70)
    labels = ['eband','dwa','teb','dwb']
    colors = ['blue','green','red','orange']
    wp = 0
    for i in range(len(axs)):
        for j in range(len(axs[i])):
            if(wp == len(maxdatacounts)):
                break
            for robot in range(len(globaldistdatas[wp])):
                axs[i][j].plot(x[wp], y[wp][robot], c=colors[robot], label=labels[robot])
            axs[i][j].legend(loc='best')
            axs[i][j].set_xlabel('Time/s',fontdict={'size':16})
            axs[i][j].set_ylabel('Distance/m',fontdict={'size':16})
            wp += 1;
    fig.autofmt_xdate()
    plt.title('Distance to the goal',fontdict={'size':20})
    plt.show()
        
def main():
    distdatalist,distlabels = load_data(distdatafile)
    pose2ddatalist,pose2dlabels = load_xyz_data(pose2ddatafile)
    veldatalist,vellabels = load_xyz_data(veldatafile)
    segtimedatalist,segtimelabels = load_segtime_data(segtimedatafile)
    waypointsdatalist = load_waypoints_data(waypointspath)
    globaldistdatalist,globalpathlabels = load_pathdist_data(globalpathdistfile)

    distdatalist = dataTimeProcess(distdatalist)
    distdatalist,maxdistdatacounts = dataFill(distdatalist)
    pose2ddatalist = dataTimeProcess(pose2ddatalist)
    pose2ddatalist,maxpose2ddatacounts = dataFill(pose2ddatalist)
    veldatalist = dataTimeProcess(veldatalist)
    veldatalist,maxveldatacounts = dataFill(veldatalist)
    globaldistdatalist = dataTimeProcess(globaldistdatalist)
    globaldistdatalist, maxglobaldistdatacounts = wpPathDataFill(globaldistdatalist, len(waypointsdatalist))
    # print(globaldistdatalist)

    dist2ObsDrawLine(distdatalist,distlabels,maxdistdatacounts)
    poseDrawLine(pose2ddatalist,maxpose2ddatacounts)
    velDrawLine(veldatalist,maxveldatacounts)
    segTimeDrawLine(segtimedatalist,segtimelabels,5)
    totlalDistDraw(pose2ddatalist,pose2dlabels,5)
    # distanceToGoalDrawLine(waypointsdatalist,pose2ddatalist,pose2dlabels,maxpose2ddatacounts)
    globalDistToGoalDrawLine(globaldistdatalist,globalpathlabels,maxglobaldistdatacounts)


if __name__ == '__main__':
    main()

from cProfile import label
import os
import math
import pandas as pd
import numpy as np
import sys
import matplotlib.pyplot as plt
from copy import deepcopy

datapath = "/home/gk/Documents/DataRecord"
distdatafile = "dist.data"
pose2ddatafile = "pose2Ddata.data"
veldatafile = "veldata.data"
segtimedatafile = "segtime.data"

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
    plt.legend(loc='best')
    plt.xlabel('Time/s')
    plt.ylabel('Dist/m')
    plt.title('Distance to the nearest obstacle',fontdict={'size':20})
    plt.show()

def poseDrawLine(datas, labels, maxdatacounts):
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
    colors = ['blue','green','red']
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
    bar_width = 0.3
    plt.bar(x=x,height=y[0],label='robot'+str(labels[0]),color='blue',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+bar_width,height=y[1],label="robot"+str(labels[1]),color='green',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+2*bar_width,height=y[2],label="robot"+str(labels[2]),color='red',alpha=0.8,width = bar_width)
    for x1,yy in enumerate(y[0]):
        plt.text(x1,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[1]):
        plt.text(x1+bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[2]):
        plt.text(x1+2*bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    plt.xlabel('waypoint')
    plt.ylabel('distance/m')
    plt.legend(loc='best')
    plt.title('Every Segment Distance Cost',fontdict={'size':20})
    plt.show()
    pass

def velDrawLine(datas,labels,maxdatacounts):
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
    colors = ['blue','green','red']
    datalabels = ['x','theta']
    for i in range(len(y[0])):
        for j in range(len(y)):
            axs[i].plot(x,y[j][i],c=colors[j],label="robot"+str(labels[j])+datalabels[i])
        axs[i].legend(loc='best')
        axs[i].set_xlabel('Time/s',fontdict={'size':16})
        axs[i].set_ylabel('linear/ m/s' if datalabels[i]=='x' else 'angular/ rad/s',fontdict={'size':16})
    fig.autofmt_xdate()
    plt.title('Velocity Information',fontdict={'size':20})
    plt.show()

def segTimeDrawLine(datas,labels,maxdatacounts):
    x = list(range(maxdatacounts))
    y = []
    for data in datas:
        data = np.array(data)
        y.append(data[:,0])
    bar_width = 0.3
    plt.bar(x=x,height=y[0],label='robot'+str(labels[0]),color='blue',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+bar_width,height=y[1],label="robot"+str(labels[1]),color='green',alpha=0.8,width = bar_width)
    plt.bar(x=np.array(x)+2*bar_width,height=y[2],label="robot"+str(labels[2]),color='red',alpha=0.8,width = bar_width)
    for x1,yy in enumerate(y[0]):
        plt.text(x1,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[1]):
        plt.text(x1+bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    for x1,yy in enumerate(y[2]):
        plt.text(x1+2*bar_width,yy+1,str(yy),ha='center',va='bottom',fontsize=12,rotation=0)
    plt.xlabel('waypoint')
    plt.ylabel('time/s')
    plt.legend(loc='best')
    plt.title('Every Segment Time Cost',fontdict={'size':20})
    plt.show()

def main():
    distdatalist,distlabels = load_data(distdatafile)
    pose2ddatalist,pose2dlabels = load_xyz_data(pose2ddatafile)
    veldatalist,vellabels = load_xyz_data(veldatafile)
    segtimedatalist,segtimelabels = load_segtime_data(segtimedatafile)

    distdatalist = dataTimeProcess(distdatalist)
    distdatalist,maxdistdatacounts = dataFill(distdatalist)
    pose2ddatalist = dataTimeProcess(pose2ddatalist)
    pose2ddatalist,maxpose2ddatacounts = dataFill(pose2ddatalist)
    veldatalist = dataTimeProcess(veldatalist)
    veldatalist,maxveldatacounts = dataFill(veldatalist)

    dist2ObsDrawLine(distdatalist,distlabels,maxdistdatacounts)
    poseDrawLine(pose2ddatalist,pose2dlabels,maxpose2ddatacounts)
    velDrawLine(veldatalist,vellabels,maxveldatacounts)
    segTimeDrawLine(segtimedatalist,segtimelabels,5)
    totlalDistDraw(pose2ddatalist,pose2dlabels,5)


if __name__ == '__main__':
    main()
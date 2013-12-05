'''
Created on 2013. 11. 18.

@author: cimple
'''

import nlopt
from numpy import *
import maya.cmds as mc
from time import time

class R2R :
    def __init__(self):
        self.count = 0       
        self.errorSum = 0.0 
        self.paramList = []
        self.paramLayer = []      
        self.paramBoundDic = {}  
        self.paramValDic = {}
        self.jntList = []
        self.activeJntDic = {}
    
    def setParamList(self, paramList):
        self.paramList = paramList
    
    def setJntList(self, jntList):
        self.jntList = jntList
        
    def setParamLayer(self):    # @todo: Now fake hierarchy making. Make hierarchy of parameters and list of them 
        self.paramLayer = []
        tempParam = []
        for param in self.paramList :
            if 'hand_ctl' in param :
                tempParam.append(param)
        self.paramLayer.append(tempParam)
        
#        for i in range(5) :   # hand - finger1_01 - finger1_02 - finger1_03 - finger2_01 - ...
#            for j in range(3) :
#                tempParam = []
#                for param in self.paramList :                    
#                    if ('finger'+str(i+1)+'_0'+str(j+1)) in param :                        
#                        tempParam.append(param)
#                self.paramLayer.append(tempParam)
        
        for i in range(5) :   # hand - [finger1_01, finger1_02, finger1_03] - [finger2_01 ...]
            tempParam = []
            for param in self.paramList :
                if 'finger'+str(i+1) in param :
                    tempParam.append(param)
            self.paramLayer.append(tempParam)

        
    def makeActiveJntDic(self):
        preJntValList = []
        for jnt in self.jntList :
            preJntVal = mc.xform(jnt, q=True, m=True, ws=True)
            preJntValList.append(preJntVal)
        for param in self.paramList :
            paramVal = mc.getAttr(param)            
            try : mc.setAttr(param, paramVal+0.5)     # @todo: Now just put in specific value (.5) but should be modified
            except : mc.setAttr(param, paramVal-0.5)  
            activeJntList = []
            for i, jnt in enumerate(jntList) :
                jntVal = mc.xform(jnt, q=True, m=True, ws=True)                
                if jntVal != preJntValList[i] : activeJntList.append(jnt)                
            self.activeJntDic[param] = activeJntList
            mc.setAttr(param, paramVal)
    
    def getActiveJntList(self, param):          
        return self.activeJntDic[param]
    
    def setParamBoundDic(self):
        for param in self.paramList:
            paramBound = [None, None]
            paramSplit = param.split('.')     
                        
            if mc.attributeQuery(paramSplit[1], re=True, node=paramSplit[0]) :
                paramBound = mc.attributeQuery(paramSplit[1], range=True, node=paramSplit[0])
            else :
                if mc.attributeQuery(paramSplit[1], mne=True, node=paramSplit[0]) :
                    paramBound[0] = mc.attributeQuery(paramSplit[1], min=True, node=paramSplit[0])
                    paramBound[1] = 1000.0
                elif mc.attributeQuery(paramSplit[1], mxe=True, node=paramSplit[0]) :
                    paramBound[0] = 1000.0
                    paramBound[1] = mc.attributeQuery(paramSplit[1], max=True, node=paramSplit[0])
                else:
                    if 'rotate' in param : paramBound = [-360.0, 360.0]
                    elif 'translate' in param : paramBound = [-1000.0, 1000.0]
                    else : paramBound = [-1000.0, 1000.0]  
                              
            self.paramBoundDic[param] = paramBound
#                    
    def setInitParamValDic(self):
        for param in self.paramList :
            self.paramValDic[param] = 0.0
            
    def initOptimize(self, paramList, jntList):  # initial settings for optimize
        self.setParamList(paramList)
        self.setJntList(jntList)
        self.setParamLayer()
        self.setParamBoundDic()   
        self.setInitParamValDic() 
        self.makeActiveJntDic()
        
    def setCtrlAttr(self, x, paramSet):       
        if(len(x) != len(paramSet)):
            print "param set - parameter size is different!"
            return;
        else:
            for i, param in enumerate(paramSet) :
                mc.setAttr(param, x[i])
    
    def getJointRotList(self, activeJntList):
        jntVal=[]
        for jnt in activeJntList :
            jntVal.append(mc.xform('char_'+jnt, q=True, rotation=True, ws=True))
        return jntVal
    
    def getGoalJointRotList(self, activeJntList):
        jntVal=[]
        for jnt in activeJntList :
            jntVal.append(mc.xform('tgt_'+jnt, q=True, rotation=True, ws=True))        
        return jntVal
        
    
    def myfunc(self, x, grad, goal, paramSet, activeJntList):
        self.count = self.count+1     
           
        self.setCtrlAttr(x, paramSet) #set parameter value
        jntValList = self.getJointRotList(activeJntList) #get current jnt value
        if(len(goal)!=len(jntValList)) : 
            print "jnt list is different!"
            return;
        error = 0.0
        for i in range(len(goal)):
            error = error + sqrt((goal[i][0]-jntValList[i][0])**2 + (goal[i][1]-jntValList[i][1])**2 + (goal[i][2]-jntValList[i][2])**2)        
        print "count : ", self.count
        return error     
    
    def optimize(self, algorithm):       
        t0 = time() 
        self.errorSum = 0.0
        for paramSet in self.paramLayer :            
            numParam = len(paramSet)
            self.count = 0            
            ##Optimization algorithm
            opt = nlopt.opt(algorithm, numParam)            
            
            ##Upper / lower bounds
            lowerBoundList = []
            upperBoundList = []
            for param in paramSet :
                bound = self.paramBoundDic[param]
                lowerBoundList.append(bound[0])
                upperBoundList.append(bound[1])            
            opt.set_lower_bounds(lowerBoundList) # set 
            opt.set_upper_bounds(upperBoundList)
            del lowerBoundList, upperBoundList
            
            #Get active joint list of the param set
            activeJntList = []            
            for param in paramSet :
                activeJntList = activeJntList + self.getActiveJntList(param)                
            activeJntList = list(set(activeJntList))   
            
            #Get goal joint value from ground-truth          
            goalJntVal = self.getGoalJointRotList(activeJntList)
            
            #Set objective function / threshold
            opt.set_min_objective(lambda x, grad: self.myfunc(x, grad, goalJntVal, paramSet, activeJntList))
            opt.set_xtol_rel(1e-10)
            
            #Optimization
            
            initVal = []
            for param in paramSet :
                initVal.append(self.paramValDic[param])            
            x = opt.optimize(initVal)
            for i, param in enumerate(paramSet) :
                self.paramValDic[param] = x[i]
            print "count : ", self.count
            print "error : ", opt.last_optimum_value()
            self.errorSum = self.errorSum +opt.last_optimum_value() 
            
        t1 = time()
            #print "optimized paramSet : ", paramSet
        print "computation time : %f" %(t1-t0)
        print "error sum : %f" %self.errorSum
    
    def makeTarget(self):
        jntVal = self.getJointRotList()
        for i in range(5):
            for j in range(3):
                mc.xform('tgt_finger'+str(i+1)+'_0'+str(j+2)+'_jnt', rotation=jntVal[i*3+j], ws=True)
        

#make paramList (hand example)
paramList = []
for i in range(5) :
    for j in range(3) :
        paramList.append('finger'+str(i+1)+'_0'+str(j+1)+'_ctl.rotateX')
        paramList.append('finger'+str(i+1)+'_0'+str(j+1)+'_ctl.rotateY')
        paramList.append('finger'+str(i+1)+'_0'+str(j+1)+'_ctl.rotateZ')
paramList.append('hand_ctl.grab')
paramList.append('hand_ctl.spread')
paramList.append('hand_ctl.rotateX')
paramList.append('hand_ctl.rotateY')
paramList.append('hand_ctl.rotateZ')

jntList = [u'hand_jnt', u'handMid_jnt', u'finger1_02_jnt', u'finger1_03_jnt', u'finger1_04_jnt', u'finger2_02_jnt', u'finger2_03_jnt', u'finger2_04_jnt', u'finger3_02_jnt', u'finger3_03_jnt', u'finger3_04_jnt', u'finger4_02_jnt', u'finger4_03_jnt', u'finger4_04_jnt', u'finger5_02_jnt', u'finger5_03_jnt', u'finger5_04_jnt']


r2r = R2R()
r2r.initOptimize(paramList, jntList)
#opt = nlopt.opt(nlopt.LN_COBYLA, numParam)            
#opt = nlopt.opt(nlopt.LN_BOBYQA, numParam)
#opt = nlopt.opt(nlopt.LN_NEWUOA_BOUND, numParam)
#opt = nlopt.opt(nlopt.LN_PRAXIS, numParam)        
#opt = nlopt.opt(nlopt.LN_NELDERMEAD, numParam)        
#opt = nlopt.opt(nlopt.LN_SBPLX, numParam)
r2r.optimize(nlopt.LN_BOBYQA)
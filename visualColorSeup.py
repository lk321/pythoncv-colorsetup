##################################################################
##################################################################
##                                                              ##
##   Author: Fernando Vidal B. , Quickcoding.io                 ##
##   E-mail: contact@quickcoding.io , vidal.fdo@gmail.com       ##
##   Title : Computer Vision and color setup                    ##
##                                                              ##
##################################################################
##################################################################

from threading import Thread
import numpy as np
import math
from PIL import Image
from numpy import array
from matplotlib import pyplot as plt
import time
import cv2
import wx
#from naoqi import ALProxy
import datetime

class VisionOne:
    def __init__(self, minVal, maxVal):
        self.colorSettingsMin = minVal.split()
        self.colorSettingsMax = maxVal.split()
        self.area = 0
        self.cap = cv2.VideoCapture(0)
        self.running = True
        self.go= True

    def ChangeColor(self, minVal, maxVal):
        self.colorSettingsMin = minVal.split()
        self.colorSettingsMax = maxVal.split()
        return self.running
 
    def cont_shape(self,img_to_contour):
        dilate = cv2.dilate(img_to_contour,None,iterations = 20)
        erode = cv2.erode(dilate,None,iterations = 15)
        contours,_= cv2.findContours(erode, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in contours:
            rect = cv2.boundingRect(c)
            if rect[2] < 5 or rect[3] < 5:
                continue
            self.area = str(cv2.contourArea(c))
            return contours

    def get_area(self):
        #print "  >>> contour Area >>>" + str(self.area)
        self.area = 0
        return self.area

    def VideoPro(self):
        while(self.go):
            screenText = False
            colorSettingsMin = np.array(self.colorSettingsMin, dtype=np.uint8)
            colorSettingsMax = np.array(self.colorSettingsMax, dtype=np.uint8)
            _,frame = self.cap.read()
            frame = cv2.GaussianBlur(frame,(5,5),0)
            frame = cv2.medianBlur(frame,5)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, colorSettingsMin, colorSettingsMax)
            res = cv2.bitwise_and(frame,frame, mask= mask)
            mask2 = cv2.cv.fromarray(mask)
            mask2 = cv2.cv.GetMat(mask2)
            moment1 = cv2.cv.Moments(mask2)
            area1= cv2.cv.GetCentralMoment(moment1,0,0)
            #print str(area1) + " <<< Area"
            opening = mask
            opening = cv2.cv.fromarray(opening)
            opening = cv2.cv.GetMat(opening)
            x1,y1,x2,y2 = (1,2,3,4)
            coord_list = [x1,y1,x2,y2]
            for x in coord_list:
                x=0
                
            if area1 > 100000:
                x2=int(cv2.cv.GetSpatialMoment(moment1,1,0)/area1)
                y2=int(cv2.cv.GetSpatialMoment(moment1,0,1)/area1)
                opening_2 = opening
                opening_2 = np.asarray(opening)
                square_find = self.cont_shape(opening_2)
                frame = np.asarray(frame)
                cv2.circle(opening_2,(x2,y2),2,(0,255,0),20)
                cv2.circle(frame,(x2,y2),2,(250,0,0),20)        
                cv2.drawContours(frame,square_find,-1,(0,255,0),3)
                frame = cv2.cv.fromarray(frame)
                frame = cv2.cv.GetMat(frame)

            frame = np.asarray(frame)
            self.get_area()
            cv2.imshow('frame',frame)
            cv2.imshow('stream',hsv)
            cv2.imshow('ColorRes',res)
            k = cv2.waitKey(25)
            
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()
        
    def online(self):
        TurnOnCamera = Thread(target = self.VideoPro)
        TurnOnCamera.start()

    def winKill(self):
        self.go= False
        
class fdo_colorSetup(wx.Frame):
    def __init__(self, parent, id):
        wx.Frame.__init__(self, parent, id, 'CAM capture & Color Set', size=(700,400))
        panel= wx.Panel(self)
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.runn= False
        self.isCamOn = False
        self.isSetRunn= False
        self.clsEvnt=''
        self.txtbox1 = wx.TextCtrl(panel, pos= (25, 280), size = (150,20))
        self.lab1 = wx.StaticText(panel, -1, "HSV Min" , (35,10))
        self.lab2 = wx.StaticText(panel, -1, "HSV Max" , (320,10))
        self.lab2 = wx.StaticText(panel, -1, "Color Tag:" , (20,250))
        
        self.slider1 = wx.Slider(panel, 100, 110, 0, 255,
                            pos=(20, 30),
                            size=(250, -1),
                            style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS ) 
        self.slider1.SetTickFreq(5,1)
        
        self.slider2 = wx.Slider(panel, 100, 50, 0, 255,
                            pos=(20, 100),
                            size=(250, -1),
                            style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS ) 
        self.slider2.SetTickFreq(5,1)

        self.slider3 = wx.Slider(panel, 100, 50, 0, 255,
                            pos=(20, 170),
                            size=(250, -1),
                            style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS ) 
        self.slider3.SetTickFreq(5,1)

        self.slider4 = wx.Slider(panel, 100, 130, 0, 255,
                            pos=(300, 30),
                            size=(250, -1),
                            style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS ) 
        self.slider4.SetTickFreq(5,1)

        self.slider5 = wx.Slider(panel, 100, 255, 0, 255,
                            pos=(300, 100),
                            size=(250, -1),
                            style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS ) 
        self.slider5.SetTickFreq(5,1)

        self.slider6 = wx.Slider(panel, 100, 255, 0, 255,
                            pos=(300, 170),
                            size=(250, -1),
                            style=wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS ) 
        self.slider6.SetTickFreq(5,1)

        btn1 = wx.Button(panel, label="Save Color", pos=(250,255), size=(80, 50))
        self.Bind(wx.EVT_BUTTON, self.HSV_Save, btn1)

        btn2 = wx.Button(panel, label="Start Capture", pos=(400,255), size=(120, 50))
        self.Bind(wx.EVT_BUTTON, self.CameraOn, btn2)

        self.OnSignal = VisionOne(self.ColorMemory(1), self.ColorMemory(2))
        
    def OnClose(self, event):
        print 'Closing'
        self.OnSignal.winKill()
        self.clsEvnt= event
        self.runn= False
        if(self.isSetRunn == False):
            event.Skip()
            self.Destroy()
        
    def ColorMemory(self, sel):
        if sel == 1:
            set_val = str(self.slider1.GetValue())+" "+ str(self.slider2.GetValue())+" "+ str(self.slider3.GetValue())+" "
            set_val.split()

        if sel == 2:
            set_val = str(self.slider4.GetValue())+" "+ str(self.slider5.GetValue())+" "+ str(self.slider6.GetValue())+" "
            set_val.split()

        return set_val
    
    def SettingsRefresh(self):
        self.isSetRunn= True
        while(self.runn):
            runn = self.OnSignal.ChangeColor(self.ColorMemory(1), self.ColorMemory(2))
        print "end"
        self.isSetRunn= False
        self.clsEvnt.Skip()
        self.Destroy()
        
    def CameraOn(self, event):
        if self.isCamOn:
            print "already running ..."
            return
        self.isCamOn= True
        self.runn = True
        self.OnSignal.online()
        self.Ref = Thread(target = self.SettingsRefresh)
        self.Ref.start()

    def HSV_Save(self, event):
        col_s = "\n\nMin:\n"+self.ColorMemory(1)+ "\nMax:\n"+self.ColorMemory(2)+"\n"
        if self.txtbox1.GetValue() in ("", " ", "Saved ..."):
            colTag= str(datetime.datetime.now().time())
        else:
            colTag= self.txtbox1.GetValue()
        col_s = "\n"+colTag+col_s
        with open("saved_c.TXT",'a') as rd:
            rd.write(col_s)
            rd.close()
        self.txtbox1.Clear()
        self.txtbox1.AppendText("Saved ...")

        
if __name__=='__main__':
    app = wx.App(redirect = False)
    frame = fdo_colorSetup(parent = None, id = -1)
    frame.Show()
    app.MainLoop()

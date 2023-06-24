
from tkinter import colorchooser
from tkinter.ttk  import Frame
from PIL import Image, ImageTk
from tkinter import *
import tkinter.filedialog as tkf
import cv2
import numpy as np
import math
from PIL import Image, ImageTk

a = Tk()
a.title("PROJECT: PICK COLOR")
a.geometry("1000x500")

def mcolor():
    color = colorchooser.askcolor()
    label = Label(text='your color',height = 2, width= 20, bg=color[1])
    label.pack()

def ncolor():
    color = colorchooser.askcolor()
    label = Label(text=color, height = 2, width= 20, bg=color[1])
    label.pack()

butt1 =  Button(text="choose color view", height= 2, width=20, command= mcolor)
butt1.place(x=100, y=100)
butt2 = Button(text="choose color code",height= 2, width=20, command= ncolor)
butt2.place(x=100, y=300)


a.mainloop()

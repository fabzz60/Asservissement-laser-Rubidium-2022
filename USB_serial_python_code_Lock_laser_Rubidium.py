# -*- coding: utf-8 -*-
# On importe Tkinter
from tkinter import Tk, StringVar, Label, Entry, Button
from tkinter import*
from functools import partial
import serial
import struct
import time
import datetime

   
   
def packIntegerAsULong(valeur):
    """Packs a python 4 byte unsigned integer to an MSP430 unsigned long"""
    return struct.pack('I', valeur)    #should check bounds

def packIntegerAsUInt(valeur):
    """Packs a python 2 byte unsigned integer to an MSP430 unsigned short"""
    return struct.pack('H', valeur)    #should check bounds

def update_label(label, stringvar):
    
    t0 = values.get()
    t17 = var.get()
    ser =serial.Serial(
    port = t0,
    #baudrate = 9600,
    baudrate = t17,
    parity = serial.PARITY_NONE, 
    stopbits = serial.STOPBITS_ONE, 
    bytesize = serial.EIGHTBITS,
    timeout = 10)
    text = stringvar.get()
    label.config(text=text)
    stringvar.set('mise à jour OK')
    t = value.get()
    ser.write(packIntegerAsULong(t))
    t2 = value2.get()
    ser.write(packIntegerAsUInt(t2))
                    
    print("virtual serial port")
    print(ser.name)
    print("baudrate")
    print(ser.baudrate)
    print("parity")
    print(ser.parity)
    print("stopbits")
    print(ser.stopbits)
    print("bytesize")
    print(ser.bytesize)
    print("timeout")
    print(ser.timeout)
              
   
root = Tk()
#root.title("Driver AOM_EOM @1.71GHz F.Wiotte 2022")
root.title("Lock Laser Rubidium paramètres DDS F.Wiotte 2022")
#root.attributes("-fullscreen", True) 
# this should make Esc exit fullscrreen, but doesn't seem to work..
#root.bind('',root.attributes("-fullscreen", False))
#root.configure(background='blue') 
root.geometry("800x400") # set explicitly window size

frame = Frame(root)
frame.pack()

bottomframe = Frame(root)
bottomframe.pack( side = BOTTOM )

bouton=Button(root, text="quitter", fg="red",font=3,command=root.destroy) # Bouton qui détruit la fenêtre
bouton.pack(side = BOTTOM)       # insère le bouton dans la fenêtre

values = StringVar(root) 
spin = Spinbox(root,values=('COM1','COM2','COM3','COM4','COM5','COM6','COM7','COM8','COM9','COM10'),fg="black",textvariable=values)
spin.pack(side = BOTTOM)

w = Label(root, text="Baud Rate and Port",fg="blue",font=("Helvetica", 10))
w.pack(side = BOTTOM)

#w = Label(root, text="1.65GHz<=Frequency AOM_EOM in Hz<=1.75GHz",fg="blue",font=("Helvetica", 12))
#w.pack(side = TOP)

var = IntVar(root)
var.set(1000000)
spin2 = Spinbox(root, from_=9600, to=1000000,increment=100,fg="black",textvariable=var)
spin2.pack(side = BOTTOM)

#value = IntVar(root) DDS CH1
value =IntVar()
value.set(0)
scale1 = Scale(frame, from_=0, to=190000000,resolution=8,fg="blue", showvalue=True,width = 15,font=('arial', 10), label='F(Hz) CH1',
variable=value, tickinterval=1, orient='h')
entry = Entry(frame, textvariable=value)
scale1.grid(row=0, column=0)
entry.grid(row=0, column=0)


#value2 = IntVar(root) DDS CH1
value2 =IntVar()
value2.set(0)
scale2 = Scale(frame, from_=0, to=4095,resolution=8,fg="blue", showvalue=True,width = 15,font=('arial', 10), label='Amp CH1',
variable=value2, tickinterval=1, orient='h')
entry = Entry(frame, textvariable=value2)
scale2.grid(row=1, column=0)
entry.grid(row=1, column=0)

#time1 = ''
#clock_lt = Label(root, font=('arial', 20, 'bold'), fg='red',bg='black')
#clock_lt.pack(side = BOTTOM)
 
text = StringVar(root)
label = Label(frame, text='')
entry_name = Entry(frame, textvariable=text)
button = Button(frame, text='SEND DATA',fg ='red',font=("Helvetica", 9),command=partial(update_label, label, text))
label.grid(column=40, row=3)
entry_name.grid(column=20, row=3)
button.grid(column=10, row=3)
 

#tick()
root.mainloop()

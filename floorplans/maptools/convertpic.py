from Tkinter import *
from tkFileDialog import askopenfilename
from PIL import Image, ImageTk
import sys

if __name__ == "__main__":
    root = Tk()
    #setting up a tkinter canvas with scrollbars
    frame = Frame(root, bd=2, relief=SUNKEN)
    frame.grid_rowconfigure(0, weight=1)
    frame.grid_columnconfigure(0, weight=1)
    xscroll = Scrollbar(frame, orient=HORIZONTAL)
    xscroll.grid(row=1, column=0, sticky=E+W)
    yscroll = Scrollbar(frame)
    yscroll.grid(row=0, column=1, sticky=N+S)
    canvas = Canvas(frame, bd=0, xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
    canvas.grid(row=0, column=0, sticky=N+S+E+W)
    xscroll.config(command=canvas.xview)
    yscroll.config(command=canvas.yview)
    frame.pack(fill=BOTH,expand=1)

    #adding the image
    File = askopenfilename(parent=root, initialdir="C:/",title='Choose an image.')
    img = ImageTk.PhotoImage(Image.open(File))
    canvas.create_image(0,0,image=img,anchor="nw")
    canvas.config(scrollregion=canvas.bbox(ALL))

    open('coordinatelist.xml', 'wb').write("List of coordinates in XML format\n")

    #function to be called when mouse is clicked
    def printcoords(event):
        open("coordinatelist.xml","ab").write('\t<Vertex p_x="%f" p_y="%f" />\n' % ((event.x/20.0), (800.0-event.y)/20.0))

        #outputting x and y coords to console
        print (event.x,event.y)
    def obstacle(event):
        if event.keysym  == 'n':
            #open("coordinatelist.xml","ab").write('\n')
            open("coordinatelist.xml","ab").write('<Obstacle closed="1" >\n')
        elif event.keysym  == 'e':
            open("coordinatelist.xml","ab").write('</Obstacle>\n\n')
        elif event.keysym == 'd':
            info = raw_input('Enter obstacle name: ')
            open("coordinatelist.xml","ab").write('<!-- %s  -->\n' % (info))
        elif event.keysym == 'q':
            sys.exit(0)

    #mouseclick event
    canvas.bind("<Button 1>",printcoords)

    #creates new obstacle
    canvas.bind_all("<KeyPress-n>",obstacle)

    #closes new obstacle
    canvas.bind_all("<KeyPress-e>",obstacle)

    #quits program
    canvas.bind_all("<KeyPress-q>",obstacle)

    #add documentation of object
    canvas.bind_all("<KeyPress-d>",obstacle)

    root.mainloop()

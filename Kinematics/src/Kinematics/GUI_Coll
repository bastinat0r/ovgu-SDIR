from GUI import GUI
from PyQt4 import QtGui,QtCore

class CollGUI(GUI,QtGui.QWidget):
    def __init__(self):     
        QtGui.QWidget.__init__(self)
        # initialize GUI
        self.initUI() 
    
    def initUI(self):         
        grid = QtGui.QGridLayout()
        # create the group of widgets at the left position
        grid.addWidget(self.createDrawGroup(), 0, 0)
        # create the group of widgets at the right position
        grid.addWidget(self.createObjGroup(), 0, 1)
        
        # set window properties
        self.setLayout(grid)
        self.resize(500,200)
        self.center()
        self.setWindowTitle('Collision')
        self.show()
        
    # create the group of widgets at the top left position
    def createDrawGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Draw')
        # use grid layout
        grid_draw = QtGui.QGridLayout()
        
        # set up button for the calculation
        self.button_draw = QtGui.QPushButton('Draw Line', self)
        
        # set up labels
        label_draw_x1 = QtGui.QLabel('x1', self)
        label_draw_y1 = QtGui.QLabel('y1', self)
        label_draw_z1 = QtGui.QLabel('z1', self)
        label_draw_x2 = QtGui.QLabel('x2', self)
        label_draw_y2 = QtGui.QLabel('y2', self)
        label_draw_z2 = QtGui.QLabel('z2', self)  
        
        # set up line edits 
        self.lineedit_draw_x1 = QtGui.QLineEdit('0', self)
        self.lineedit_draw_y1 = QtGui.QLineEdit('0', self)  
        self.lineedit_draw_z1 = QtGui.QLineEdit('0', self)
        self.lineedit_draw_x2 = QtGui.QLineEdit('0', self) 
        self.lineedit_draw_y2 = QtGui.QLineEdit('0', self) 
        self.lineedit_draw_z2 = QtGui.QLineEdit('0', self)
        
        # trigger function on clicked
        QtCore.QObject.connect(self.button_draw, QtCore.SIGNAL("clicked()"), self.buttonDrawClicked)
        
        # add the widgets to the grid layout
        grid_draw.addWidget(label_draw_x1, 0, 0)
        grid_draw.addWidget(self.lineedit_draw_x1, 0, 1)
        grid_draw.addWidget(label_draw_y1, 1, 0)
        grid_draw.addWidget(self.lineedit_draw_y1, 1, 1)
        grid_draw.addWidget(label_draw_z1, 2, 0)
        grid_draw.addWidget(self.lineedit_draw_z1, 2, 1)
        grid_draw.addWidget(label_draw_x2, 0, 2)
        grid_draw.addWidget(self.lineedit_draw_x2, 0, 3)
        grid_draw.addWidget(label_draw_y2, 1, 2)
        grid_draw.addWidget(self.lineedit_draw_y2, 1, 3)
        grid_draw.addWidget(label_draw_z2, 2, 2)
        grid_draw.addWidget(self.lineedit_draw_z2, 2, 3)
        grid_draw.addWidget(self.button_draw, 2,4)
        
        # set the grid layout for the group
        group_box.setLayout(grid_draw)
        
        return group_box
        
    # create the group of widgets at the top left position
    def createObjGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Create Objects (in mm)')
        # use grid layout
        grid_obj = QtGui.QGridLayout()
        
        # set up labels
        label_obj_x = QtGui.QLabel('x', self)
        label_obj_y = QtGui.QLabel('y', self)
        label_obj_z = QtGui.QLabel('z', self)
        label_obj_lx = QtGui.QLabel('laenge_x', self)
        label_obj_ly = QtGui.QLabel('laenge_y', self)
        label_obj_lz = QtGui.QLabel('laenge_z', self)  
        
        # set up line edits 
        self.lineedit_obj_x = QtGui.QLineEdit('0', self)
        self.lineedit_obj_y = QtGui.QLineEdit('0', self)  
        self.lineedit_obj_z = QtGui.QLineEdit('0', self)
        self.lineedit_obj_lx = QtGui.QLineEdit('0', self) 
        self.lineedit_obj_ly = QtGui.QLineEdit('0', self) 
        self.lineedit_obj_lz = QtGui.QLineEdit('0', self)
        
        # set up checkbox
        self.checkbox_obj = QtGui.QCheckBox('Random', self)
        
        # set up button for the calculation
        self.button_obj = QtGui.QPushButton('Create Object', self)
        
        # trigger function on clicked
        QtCore.QObject.connect(self.button_obj, QtCore.SIGNAL("clicked()"), self.buttonObjClicked)
        
        # add the widgets to the grid layout
        grid_obj.addWidget(label_obj_x, 0, 0)
        grid_obj.addWidget(self.lineedit_obj_x, 0, 1)
        grid_obj.addWidget(label_obj_y, 1, 0)
        grid_obj.addWidget(self.lineedit_obj_y, 1, 1)
        grid_obj.addWidget(label_obj_z, 2, 0)
        grid_obj.addWidget(self.lineedit_obj_z, 2, 1)
        grid_obj.addWidget(label_obj_lx, 0, 2)
        grid_obj.addWidget(self.lineedit_obj_lx, 0, 3)
        grid_obj.addWidget(label_obj_ly, 1, 2)
        grid_obj.addWidget(self.lineedit_obj_ly, 1, 3)
        grid_obj.addWidget(self.checkbox_obj, 1, 4)
        grid_obj.addWidget(label_obj_lz, 2, 2)
        grid_obj.addWidget(self.lineedit_obj_lz, 2, 3)
        grid_obj.addWidget(self.button_obj, 2, 4)
        
        # set the grid layout for the group
        group_box.setLayout(grid_obj)
        
        return group_box
    
    # function is called when the calculate IK button is clicked
    def buttonDrawClicked(self):
        # prefix for parsing
        prefix = "DRA#"   
        # get values and convert QString to string
        values = str(self.lineedit_draw_x1.text()+";"+self.lineedit_draw_y1.text()+";"+self.lineedit_draw_z1.text()+";"+self.lineedit_draw_x2.text()+";"+self.lineedit_draw_y2.text()+";"+self.lineedit_draw_z2.text()) 
        # send data
        self.dataTransfer(prefix+values)    
        
    # function is called when the calculate IK button is clicked
    def buttonObjClicked(self):
        # prefix for parsing
        prefix = "OBJ#"   
        sufix = ""
        # get values and convert QString to string
        values = str(self.lineedit_obj_x.text()+";"+self.lineedit_obj_y.text()+";"+self.lineedit_obj_z.text()+";"+self.lineedit_obj_lx.text()+";"+self.lineedit_obj_ly.text()+";"+self.lineedit_obj_lz.text()) 
        if self.checkbox_obj.isChecked():
            sufix = "#R"
        # send data
        self.dataTransfer(prefix+values+sufix)      

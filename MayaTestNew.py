
from __future__ import print_function;

import viz
import vizact
import vizcam
import vizshape
import vizinfo

viz.setMultiSample(4)
viz.fov(60)
viz.go()

class Torch:
	
	def __init__(self, start, mylight):
		self.start = start
		self.mylight = mylight
		self.setup()
		
		return ( None );
	#tini

	
	def setup(self):
		#Disable the headlight on the room
		viz.MainView.getHeadLight().disable()
		
		#Add a light to the empty node
		self.mylight = viz.addLight()

		#Set the light parameters
		self.mylight.position(0,0,0)
		self.mylight.direction(0,0,1)
		self.mylight.spread(15)
		self.mylight.intensity(self.start)
		self.mylight.spotexponent(100)
		self.mylight.setPosition([0,1,0])
		
		#Link the torch to the MainView
		viz.link(viz.MainView, self.mylight)
		return None
	#putes	

	## CALLBACK FUNCTION FOR CHANGING BRIGHTNESS
	def changeBrightness( self, amount ):
		
		prior = self.start;
		
		self.start += amount;
		
		if ( self.start < 0 ):
			
			self.start = 0;
			
		#fi
		
		if ( self.start >= 7 ):
			
			self.start = 7
		
		#fi
		
		self.mylight.intensity( self.start );
		
		print( 'Starting intensity is: ' +
			   str( prior ) +
			   'Final is: ' +
			   str( self.start ), end= "\n" );
			   
		return ( None );
		
	#ssenthgirbegnahc

#hcrot

################################# Script ##############################################
#Test Object
torchy = Torch(start= 7, mylight= 0)

#Add special tesselated room
wall = viz.addChild('E:\Users\VicMcG\Documents\maya\projects\default\scenes\TestingMaze2.obj')
wall.setPosition([0, -70, 0])

viz.MainView.setPosition([0,0,0])

############################## Keyboard ###############################################
#Torch keydown functions
vizact.onkeydown('l', torchy.mylight.enable)
vizact.onkeydown('k', torchy.mylight.disable)

#Torch keydown functions
vizact.onkeydown('v', torchy.changeBrightness, 1)
vizact.onkeydown('c', torchy.changeBrightness, -1)

#Screen recording keydown functions
vizact.onkeydown('p', viz.window.toggleRecording, 'test.avi')
vizact.onkeydown('o', viz.window.stopRecording)

################################ Wii #################################################
#Wiimote extension
wii = viz.add('wiimote.dle')
wiimote = wii.addWiimote()
wiimote.led = wii.LED_1 | wii.LED_4

#when program closes/Wiimote disconnects
vizact.onexit(wiimote.remove)

##mask Wiimote 1 & 2 to turn on or off the light
vizact.onsensordown(wiimote, wii.BUTTON_1, torchy.mylight.enable)
vizact.onsensordown(wiimote, wii.BUTTON_2, torchy.mylight.disable)

#mask Wiimote + & - to dim or brighten the light to a limit
vizact.onsensordown(wiimote, wii.BUTTON_MINUS, torchy.changeBrightness, -3)
vizact.onsensordown(wiimote, wii.BUTTON_PLUS, torchy.changeBrightness, 3)

############################### Mocap #################################################
from mocapInterfaceNew import mocapInterface
phaseSpaceIP = '192.168.1.230'
owlParamMarkerCount = 20

# Dir where textures and rb files are located
phaseSpaceFilePath = 'Resources/'

# Rb files in phaseSpaceFilePath
rigidFileNames_ridx = ['hmd-nvis.rb','torch.rb']

# Shapes defined in rigidbody._createVizNode()
# Look there to see what shapes are accepted, or add more shapes
rigidBodyShapes_ridx = ['sphere','cylinder']

# Sizes must correspond to parameters for the vizshape in rigidBodyShapes_ridx 
rigidBodySizes_ridx = [[.1],[.15,.03]]

# Is the rigid body visible on startup?
rigidBodyToggleVisibility_ridx = [0,1]

# Start up the motion capture server
mocap = mocapInterface(phaseSpaceIP, phaseSpaceFilePath,
		owlParamMarkerCount, rigidFileNames_ridx, rigidBodyShapes_ridx,
		rigidBodySizes_ridx, rigidBodyToggleVisibility_ridx)

torchRigid = mocap.returnPointerToRigid('torch')
torchLink = viz.link( torchRigid.vizNode, torchy.mylight )
torchLink.preAxisAngle([0,1,0,-90])
vizact.onkeydown('h', mocap.resetRigid,'torch')

############################# Camera Moves ############################################

""" camera game like mode """
camera = vizcam.FlyNavigate(
	forward = 'w',
	backward = 's',
	left = 'a',
	right = 'd',
	up = ' ',
	down = viz.KEY_CONTROL_L
)
camera.MOVE_SPEED = 50
camera.sensitivity(5.0, 2.0)

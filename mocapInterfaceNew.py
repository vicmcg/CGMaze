import viz
import vizshape
import math
from OWL import *
from os import rename

# Marker ID returned by owlGetMarkers() depends on 
# the ID of the rigid body to which it is attached
# and the the rigid_marker number (from zero to N, where N = num of markers attached to rigid body)
# This takes tracker num and rigid-marker id and converts.

def rigidXMarkerID(tracker, index):
	
	## BIT SHIFT LEFT BY 12 BITS IS BINARY MULTIPLICATION BY (2 ** 12)
	return ( ( tracker << 12 ) | (index) );
	
#fed

class rigidBody(viz.EventClass):
	
	def __init__(self,trackerIdx,filePath,fileName,vizNodeShape,vizNodeSize,isVisible):
		
		self.filePath = filePath
		self.fileName = fileName
		
		self.trackerIdx = trackerIdx
		self.markerID_midx = []
		self.markerPos_midx_localXYZ = []
		
		self.isVisible = isVisible
		self.vizNode = 0
		self.physNode = 0
		
		self.vizNodeLink = 0
		self.vizNodeShape = vizNodeShape
		
		if( type(vizNodeSize) is list and len(vizNodeSize) == 1):
			self.vizNodeSize  = vizNodeSize[0]
		else:
			self.vizNodeSize  = vizNodeSize
		
		self.serverData = []
		
		# Create rigid tracker		
		self._loadDefaults()		
			
		# Create rigidTracker
		owlTrackeri( self.trackerIdx, OWL_CREATE, OWL_RIGID_TRACKER );
		
		# Populate rigidTracker 
		for i in range( len(self.markerID_midx) ):
			
			# set markers
			owlMarkeri( MARKER(self.trackerIdx, i), OWL_SET_LED, self.markerID_midx[i] )
			# set marker positions
			owlMarkerfv(MARKER(self.trackerIdx, i), OWL_SET_POSITION, self.markerPos_midx_localXYZ[i])
		
		# Activate tracking for rigidTracker
		owlTracker( self.trackerIdx, OWL_ENABLE )
			
		
		# Flush requests and check for errors
		if not owlGetStatus():    
			#owl_print_error("error in point tracker setup", owlGetError())
			errorCode = owlGetError();
			print("Error in point tracker setup.", errorCode)
			return 0
		
		self._createVizNode(self.vizNodeShape,self.vizNodeSize)
		
						
	def _loadDefaults(self):

		import os.path
		openfile = [];
		openfile = open( self.filePath + self.fileName, 'r' );
		
		lineData = openfile.readlines();

		parsedData = [];

		markerID = [];
		markerPos = [];
		
		count = 0;
		
		for currentLine in lineData:
			
			tempLineDataList = currentLine.split(',');
			
			markerID.append( int(tempLineDataList[0]) );
			
			markerPos.append( map( float, tempLineDataList[1].split() ) );
			
			count += 1;
			
		#rof
			
		self.markerID_midx = markerID
		self.markerPos_midx_localXYZ = markerPos
		
		openfile.close()
		
		
		print 'Mocap: Read ' + str(count) + ' lines from the rigid body file.'
		if count == 0: print 'This is likely to cause OWL.init() to fail'
		
	def _createVizNode(self,shape,size):
		
		if(shape == 'box' ):
			
			print 'Making box vizNode'
			
			if( type(size) == float or len(size) !=3): 
				print '**********Invalid size for ' + self.fileName
				print 'Check rigidBodySizesString.  Expected 3 val for box: height,width,length.'
				print 'Got: ' + str(size)
				import winsound
				winsound.Beep(1000,250)

			self.vizNode = vizshape.addBox(size,alpha = 0.7,color=viz.RED)				
			return
			
		elif(shape == 'sphere'):
		
			if( type(size) != float):  # accept a float
				
				print '**********Invalid size for ' + self.fileName
				print 'Check rigidBodySizesString.  Expected 1 val for sphere: radius'
				print 'Got: ' + str(size)
				import winsound
				winsound.Beep(1000,250)
			
			print 'Making sphere vizNode'
			self.vizNode = vizshape.addSphere(radius = float(size), alpha = 0.7,color=viz.BLUE)
			
		elif(shape == 'cylinder'):
			
			
			if( type(size) == float or len(size) !=2): 
				
				print '**********Invalid size for ' + self.fileName
				print 'Check rigidBodySizesString.  Expected 2 val for cylinder: height,radius'
				print 'Got: ' + str(size)
				import winsound
				winsound.Beep(1000,250)
				
			print 'Making cylinder vizNode'
			# height, rad,
			
			self.vizNode = vizshape.addCylinder(height=size[0],radius=size[1], alpha = 0.3,color=viz.YELLOW,axis=vizshape.AXIS_X)
			#self.vizNode.setAxisAngle(1,0,0,90)
			
		if( self.vizNode ) :
			
			self.vizNode.disable(viz.CULL_FACE)
			
			if( self.isVisible ):
				self.vizNode.visible( viz.ON ) #Make the object visible.
			else:
				self.vizNode.visible( viz.OFF ) #Make the object invisible.
				print self.fileName + ' vizNode created, but not set as visible'
				
			
			self.physNode = self.vizNode.collideMesh()
			
		else:
			print 'Problem creating viz node'
			import winsound
			winsound.Beep(1000,250)
		
		
	def updateVizNode(self,pos_XYZ,quat_XYZW):
	
		transformMatrix = viz.Transform()
		transformMatrix.setQuat(quat_XYZW)
		transformMatrix.postTrans(pos_XYZ)
		transformMatrix.postAxisAngle(0,1,0,90)
		self.vizNode.setMatrix(transformMatrix)
		
	def getMarkerPositions(self,allMarkers_midx):
		
		# Build list of updated marker positions in newPosWorldCoords
		newPos_midx_GlobalXYZ = [];
		
		for oldMarkerIdx in range( 0, len(self.markerID_midx), 1 ):

			for newMarkerIdx in range( 0, len(allMarkers_midx), 1 ):
				
				# Confused?  See rigidXMarkerID macro at top!
				oldRigidIDConverted = rigidXMarkerID( self.trackerIdx, oldMarkerIdx );
				
				if( allMarkers_midx[newMarkerIdx].id == oldRigidIDConverted ):
					
					newPos_midx_GlobalXYZ.append( [ allMarkers_midx[newMarkerIdx].x, 
												allMarkers_midx[newMarkerIdx].y,
												allMarkers_midx[newMarkerIdx].z ] );
												
				#fi
				
			#rof
			
		#rof
		
		if( len(newPos_midx_GlobalXYZ) < len(self.markerID_midx ) ):
			
			print "getMarkerPositions:  Error. Could not see all markers."
			
			for i in range(len(allMarkers_midx)):
				print ("  Visible Marker IDs: " + str(allMarkers_midx[i].id) );
			
			return -1
			#rof
		else:
			return newPos_midx_GlobalXYZ
	
	def saveNewDefaults(self):
		
		fileObject = open(self.filePath + 'temp.rb','w')#self.fileName , 'w')
		
		# Get old marker id's and new positions
		oldRigidID_midx = self.markerID_midx
		newRigidPos_midx_xyz = self.markerPos_midx_localXYZ
		
		for idx in range(len(newRigidPos_midx_xyz )):
			posString = str(newRigidPos_midx_xyz[idx][0]) + ' ' + str(newRigidPos_midx_xyz[idx][1]) + ' ' + str(newRigidPos_midx_xyz[idx][2]);
			newLine = str(oldRigidID_midx[idx]) + ', ' + posString + '\n'
			fileObject.write(newLine);
			
		fileObject.close();
		
		#from shutil import move
		#move(self.filePath + 'temp.rb', self.filePath + self.fileName)

		from os import remove
		from shutil import move

		remove(self.filePath + self.fileName)
		move(self.filePath + 'temp.rb', self.filePath + self.fileName)

		print "Rigid body definition written to file"	
		
	def resetRigid( self, allMarkers_midx ):
		
		#newPos_midx_GlobalXYZ = self.getMarkerPositions( allMarkers_midx )
		
		# Build list of updated marker positions in newPosWorldCoords
		newPos_midx_GlobalXYZ = [];
		
		# Find markers that belong to the rigid tracker.
		# Collect their position in newPos_midx_GlobalXYZ
		for oldMarkerIdx in range( 0, len(self.markerID_midx), 1 ):
			for newMarkerIdx in range( 0, len(allMarkers_midx), 1 ):
				
				# rigidXMarkerID macro defined at top of file!
				oldRigidIDConverted = rigidXMarkerID( self.trackerIdx, oldMarkerIdx );
				
				if( allMarkers_midx[newMarkerIdx].id == oldRigidIDConverted ):
					
					newPos_midx_GlobalXYZ.append( [ allMarkers_midx[newMarkerIdx].x, 
												allMarkers_midx[newMarkerIdx].y,
												allMarkers_midx[newMarkerIdx].z ] );
												
				#fi
				
			#rof
			
		#rof
		
		
			
		if( len(newPos_midx_GlobalXYZ) < len(self.markerID_midx ) ):
			
			print "ResetRigid:  Error. Could not see all markers."
			
			for i in range(len(allMarkers_midx)):
				print ("  Visible Marker IDs: " + str(allMarkers_midx[i].id) );
			#rof
			
			return
		
		####################################################################################
		##  Here's where we set the COM / center of rotatnoi for the rigid body
		
		if( self.fileName.find('nvis')> -1 ):
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using mean of first and last markers as COM'
			
			# calculate the center of the rigid body using the first and last marker
			center_GlobalXYZ = [(newPos_midx_GlobalXYZ[0][0] + newPos_midx_GlobalXYZ[-1][0])/2,
					  (newPos_midx_GlobalXYZ[0][1] + newPos_midx_GlobalXYZ[-1][1])/2,
					  (newPos_midx_GlobalXYZ[0][2] + newPos_midx_GlobalXYZ[-1][2])/2]
					  
		elif( self.fileName.find('oculus')> -1 ):
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using mean of markers 2 and 3 as COM'
			
			# calculate the center of the rigid body using the first and last marker
			# FOr the oculus, the eyes are 2.5 cm behind the front of the helmet
			
			center_GlobalXYZ = [(newPos_midx_GlobalXYZ[3][0] + newPos_midx_GlobalXYZ[2][0])/2,
					  (newPos_midx_GlobalXYZ[3][1] + newPos_midx_GlobalXYZ[2][1])/2,
					  (-2.5+newPos_midx_GlobalXYZ[3][2] + newPos_midx_GlobalXYZ[2][2])/2]
			
			
			#self.vizNode.setCenter(0,0,0)
					  
		else:
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using mean of all markers as COM'
			
			sum = []
			center_GlobalXYZ = [0,0,0]
			for dim in range(3):
				sum = 0
				
				for mIdx in range(len(newPos_midx_GlobalXYZ)):
					sum = sum+newPos_midx_GlobalXYZ[mIdx][dim]
				
				center_GlobalXYZ[dim] = sum / len(newPos_midx_GlobalXYZ)
				
#			center_GlobalXYZ = [sum(newPos_midx_GlobalXYZ[:-1][0])/len(newPos_midx_GlobalXYZ),
#						  sum(newPos_midx_GlobalXYZ[:][1])/len(newPos_midx_GlobalXYZ),
#						  sum(newPos_midx_GlobalXYZ[:][2])/len(newPos_midx_GlobalXYZ)]

		# Convert to rigid body coordinates, relative to center
		newPosRigidCoords_mIdx_LocalXYZ = []
		
		for i in range( 0, len(newPos_midx_GlobalXYZ), 1 ):
			
			newPosRigidCoords_mIdx_LocalXYZ.append( [ newPos_midx_GlobalXYZ[i][0]-center_GlobalXYZ[0],
										newPos_midx_GlobalXYZ[i][1]-center_GlobalXYZ[1],
										newPos_midx_GlobalXYZ[i][2]-center_GlobalXYZ[2] ] );
										
		#rof
			
		# Update rigid body definition on the owl server
		# and in this rigid body
		owlTracker( self.trackerIdx, OWL_DISABLE );
		
		for i in range( 0, len(newPosRigidCoords_mIdx_LocalXYZ), 1 ):
			
			owlMarkerfv( MARKER(self.trackerIdx, i), OWL_SET_POSITION, newPosRigidCoords_mIdx_LocalXYZ[i] )
			self.markerPos_midx_localXYZ[i] = newPosRigidCoords_mIdx_LocalXYZ[i];
			
		#rof
			
		owlTracker( self.trackerIdx, OWL_ENABLE );
		
		if not owlGetStatus():
			
			print ( "ResetRigid: Could not enable rigid body. OwlGetStatus returned: ", owlGetError() );
			
			exit();
		
		#fi
		
		print "Rigid body definition updated on server."

	def toggleVisibility(self):
		
		if(self.isVisible): 
			
			print 'Rigid body no longer visible'
			self.isVisible = 0
			self.vizNode.visible( viz.OFF ) #Make the object visible.
			
		elif self.isVisible == 0:		
			print 'Rigid body visible'
			self.isVisible = 1
			self.vizNode.visible( viz.ON ) #Make the object visible.
	
	def rotateRigid(self,rotateByDegs_XYZ):

		# create a temp vizard 3d object 
		# rotate, use vertices to redefine rigid object
		# maybe that phasespace coordinate system does not match vizard coordinate system
		
		viz.startLayer(viz.POINTS)
		for idx in range(len(self.markerPos_midx_localXYZ)):
			viz.vertex(self.markerPos_midx_localXYZ[idx][0],self.markerPos_midx_localXYZ[idx][1],self.markerPos_midx_localXYZ[idx][2])
		
		tempRigidObject = viz.endLayer()
		tempRigidObject.visible( viz.OFF ) #Make the object invisible.
		
		# Update rigid body definition on the owl server
		#tempRigidObject.setEuler( rotateByDegs_XYZ,viz.RELATIVE ) #,viz.RELATIVE)		
		
		tempRigidObject.setQuat(rotateByDegs_XYZ,viz.ABS_GLOBAL)
		
		owlTracker(self.trackerIdx,OWL_DISABLE)

		count = 0
		for i in xrange(len(self.markerID_midx)):
			owlMarkerfv(MARKER(self.trackerIdx, i), OWL_SET_POSITION, tempRigidObject.getVertex(i,viz.RELATIVE))
			self.markerPos_midx_localXYZ[count] = tempRigidObject.getVertex(i,viz.RELATIVE)
			count +=1
			
		owlTracker(self.trackerIdx, OWL_ENABLE);
		
		tempRigidObject.remove() #Remove the object.
		
	
		
class rigidHMD(rigidBody):
	def __init__(self,trackerIdx,filePath,fileName,vizNodeShape,vizNodeSize,isVisible):
		
		print 'Initializing HMD'
		
		super(rigidHMD,self).__init__(trackerIdx,filePath,fileName,vizNodeShape,vizNodeSize,isVisible);
		#self.vizNodeLink = viz.link(self.vizNode,viz.MainView)
		
		self.showCylinders = 0
		
		########################################################
		########################################################
		##  Parameters of the eyes and eye-probes
		
		self.iod = viz.MainWindow.getIPD()
		self.eyeProbeDistance = .02
		self.eyeOffsetDegs = self.eyeProbeDistance * math.tan(math.radians(self.iod/2));
		
		self.eyeProbeCylLength = .6
		self.eyeGridSize = .003
		self.eyeProbeRadius = viz.MainWindow.getIPD()/4
		
		self.probeBarWidthMM = .002;
		
		# During calibration, the near plane is drawn self.planeSeperation
		# mm in front of the far plane
		self.planeSeperation = .01;
		self.calibLineWidthDegs = 2;
		self.tempAngle= 0;
		
		self.createCalibrationCylinders()
		
	def createCalibrationCylinders(self):
		
		##############################################################
		##############################################################
		## left eye and calib grid
		
		self.leftEyeProbeCylinder = vizshape.addCylinder(height=self.eyeProbeCylLength,radius=self.eyeProbeRadius,top=False,bottom=False,axis = vizshape.AXIS_X,color=viz.BLUE)
		self.leftEyeProbeCylinder.disable(viz.CULL_FACE)
		self.leftEyeProbeCylinder.visible( viz.OFF )
		
		self.leftEyeCylinderBack = vizshape.addCircle(self.eyeProbeRadius,axis=vizshape.AXIS_X,color=viz.GREEN)
		self.leftEyeCylinderBack.disable(viz.CULL_FACE)
		self.leftEyeCylinderBack.setParent(self.leftEyeProbeCylinder)
		self.leftEyeCylinderBack.setPosition(self.eyeProbeCylLength/2,0,0) #self.leftEyeProbeCylinder)
		
		self.lGridVertBar = vizshape.addBox([self.probeBarWidthMM,self.eyeProbeRadius*2,self.probeBarWidthMM])
		self.lGridVertBar.setParent(self.leftEyeCylinderBack)

		##############################################################
		##############################################################
		## Draw right eye components 
		
		self.rightEyeProbeCylinder = vizshape.addCylinder(height=self.eyeProbeCylLength,radius=self.eyeProbeRadius,top=False,bottom=False,axis = vizshape.AXIS_X,color=viz.BLUE)
		self.rightEyeProbeCylinder.disable(viz.CULL_FACE)
		self.rightEyeProbeCylinder.visible( viz.OFF )
		
		self.rightEyeCylinderBack = vizshape.addCircle(self.eyeProbeRadius,axis=vizshape.AXIS_X,color=viz.GREEN)
		self.rightEyeCylinderBack.disable(viz.CULL_FACE)
		self.rightEyeCylinderBack.setParent(self.rightEyeProbeCylinder)
		self.rightEyeCylinderBack.setPosition(self.eyeProbeCylLength/2,0,0) #self.leftEyeProbeCylinder)
		
		self.rGridHorzBar = vizshape.addBox([self.probeBarWidthMM,self.probeBarWidthMM,self.eyeProbeRadius*2])
		self.rGridHorzBar.setParent(self.rightEyeCylinderBack)
		
		
#	def updateVizNode(self,pos,rot):
#		
#		# Call the update function of parent class, rigidBody
#		# This calls rigidBody.updateVizNode(pos,rot)
#		super(rigidHMD,self).updateVizNode(pos,rot)
#	
#		# Now, if eyes are draw, change their position
#		if( self.showCylinders ):
#			
#			
#			headPos_XYZ = self.vizNode.getPosition();
#			
#			# move the eyes self.eyeProbeDistance up the Z axis, and 
#			#self.leftEyeCylinderBack.setPosition(headPos_XYZ[0]-self.iod/2,headPos_XYZ[1],headPos_XYZ[2]-self.eyeProbeDistance)
#			
##			self.leftEyeProbeCylinder.setPosition(headPos_XYZ[0]+self.eyeProbeDistance+self.eyeProbeCylLength/2,headPos_XYZ[1],headPos_XYZ[2]+self.iod/2)
##			self.leftEyeProbeCylinder.setEuler([90,0,0])
##			
##			self.leftEyeCylinderBack.setPosition(0,0,self.planeSeperation)
##			
##			self.lGridVertBar.setPosition([0,0,0])
#			
#			################################################
#			################################################
##			
##			# Set right plane positions
##			#self.rightEyeCylinderBack.setPosition(headPos_XYZ[0]+self.iod/2,headPos_XYZ[1],headPos_XYZ[2]-self.eyeProbeDistance)
##			self.rightEyeProbeCylinder.setPosition(headPos_XYZ[0]+self.eyeProbeDistance,headPos_XYZ[1],headPos_XYZ[2]-self.iod/2)
##			self.rightEyeProbeCylinder.setEuler([90,0,0])
##			
##			self.rightEyeCylinderBack.setPosition(0,0,self.planeSeperation)
##						
##			self.rGridHorzBar.setPosition([0,0,0])
#			
	def toggleCalibrationCylinders(self):
		
		if(self.showCylinders): 
			
			self.showCylinders = 0
			self.leftEyeProbeCylinder.visible( viz.OFF ) #Make the object visible.
			self.rightEyeProbeCylinder.visible( viz.OFF ) #Make the object visible.
			# reset clipping plane to previous values
			print 'Eye probes off, head on'
			
		elif self.showCylinders == 0:		
			
			self.showCylinders = 1
			self.leftEyeProbeCylinder.visible( viz.ON ) #Make the object visible.
			self.rightEyeProbeCylinder.visible( viz.ON ) #Make the object visible.
			#viz.MainWindow.clip(0.001 ,1)
			print 'Eye probes on, head off'
			
			headPos_XYZ = self.vizNode.getPosition();
			self.leftEyeProbeCylinder.setPosition(0,headPos_XYZ[1],(viz.MainWindow.getIPD()/2)) #-.06/4)#headPos_XYZ[2]-self.iod/4)
			self.rightEyeProbeCylinder.setPosition(0,headPos_XYZ[1],(-viz.MainWindow.getIPD()/2))
			
	def changeIOD(self,increment):
		
		self.iod += increment
		if(self.iod <= .01):
			#NOPE
			
			self.iod -= increment
			return
		else:
			self.iod += increment
		
		self.eyeOffsetDegs = math.atan( math.radians( self.iod/2/self.eyeProbeDistance ))
		
		self.leftEyeCylinderBack.setPosition(-self.iod/2,0,self.eyeProbeDistance)	
		self.rightEyeCylinderBack.setPosition(self.iod/2,0,self.eyeProbeDistance)
		
		print 'Offset is: ' + str( self.eyeOffsetDegs )
			
	def changeEyeProbeDistance(self,increment):
		
		# Preserves IOD by allowing angle to change
		self.eyeProbeDistance += increment
		print 'Distance is: ' + str( self.eyeProbeDistance )
		self.eyeOffsetDegs = math.atan( math.radians( self.iod/2/self.eyeProbeDistance ))
		print 'Offset is: ' + str( self.eyeOffsetDegs )
		
		#self.iod = self.eyeProbeDistance * math.tan( math.radians( self.eyeOffsetDegs ))
		if( self.eyeProbeDistance > 1 ): self.eyeProbeDistance = 1;
		
		if( self.eyeProbeDistance < -.5 ): self.eyeProbeDistance = .05;
		
		self.leftEyeCylinderBack.setPosition(-self.iod/2,0,self.eyeProbeDistance)	
		self.rightEyeCylinderBack.setPosition(self.iod/2,0,self.eyeProbeDistance)
	
	def getHeadOrientation(self):
		
		# is server data up to date?
		# is rotateRigid working correctly?
		# 
		
		# Quaternion
		rot = [ self.serverData.pose[4], self.serverData.pose[5], -self.serverData.pose[6], -self.serverData.pose[3] ];
		#[0.002087714894559653, 0.7072209410999575, -0.0021176456789796234, 0.7069863488731505]
		
		transformMatrix = viz.Transform()
		transformMatrix.setQuat(rot)
		# Not sure why I need to rotate 180 degs here.
		transformMatrix.postAxisAngle(0,1,0,90) 
		#import vizmat
		#euler_XYZ = vizmat.QuatToEuler(transformMatrix.getQuat())
		quat_XYZW = transformMatrix.getQuat()
		quat_XYZW[3] = -quat_XYZW[3]
		print str(quat_XYZW)
		self.rotateRigid(quat_XYZW)
		#self.rotateRigid([-euler_XYZ[0],-euler_XYZ[1],-euler_XYZ[2]])
		
		
		
		

		#self.rotateRigid([-orientation_XYZ[2],-orientation_XYZ[1],-orientation_XYZ[1]])
		
		#newPos_midx_GlobalXYZ = self.getMarkerPositions( allMarkers_midx )
		
		#(newPos_midx_GlobalXYZ[0] - newPos_midx_GlobalXYZ[-1])
#		
#		viz.startLayer(viz.POINTS)
#		
#		for idx in range(len(newPos_midx_GlobalXYZ)):
#			#viz.vertex(newPos_midx_GlobalXYZ[idx][0],newPos_midx_GlobalXYZ[idx][1],newPos_midx_GlobalXYZ[idx][2])
#			
#			viz.vertex(self.markerPos_midx_localXYZ[idx][0],self.markerPos_midx_localXYZ[idx][1],self.markerPos_midx_localXYZ[idx][2])
#		
#		tempRigidObject = viz.endLayer()
#		tempRigidObject.visible( viz.OFF ) #Make the object invisible.
		
#		#############################
#		
#		# Update rigid body definition on the owl server
#		# and in this rigid body
#		owlTracker( self.trackerIdx, OWL_DISABLE )
#		
#		for i in range( 0, len(newPosRigidCoords_mIdx_LocalXYZ), 1 ):
#			
#			owlMarkerfv( MARKER(self.trackerIdx, i), OWL_SET_POSITION, newPosRigidCoords_mIdx_LocalXYZ[i] )
#			self.markerPos_midx_localXYZ[i] = newPosRigidCoords_mIdx_LocalXYZ[i];
#			
		#rof
#			
#		owlTracker( self.trackerIdx, OWL_ENABLE );
#		
#		if not owlGetStatus():
#			
#			print ( "ResetRigid: Could not enable rigid body. OwlGetStatus returned: ", owlGetError() );
#			
#			exit();
			
class mocapInterface(viz.EventClass):
	
	def __init__(self, phaseSpaceIP,phaseSpaceFilePath,owlParamMarkerCount,rigidFileNames_ridx,
	rigidBodyShapes_ridx,rigidBodySizes_ridx,rigidBodyToggleVisibility_ridx):
		
		viz.EventClass.__init__(self)
		
		#self.config = config;
		self.origin = []
		self.scale = []
		self.serverAddress = []
		self.showCylinders = 0
		self.rigidHMDIdx = -1
		
		self.allRigids_ridx = [];
		self.allMarkers_midx = [];
		self.allTrackers_tidx = []; # A method for keeping track of tracker numbers
		
		self.frame 			   		= 0
		self.last_tick 		   		= 0
		self.off 			   		= True
		self.markersUsedInRigid 	= [];
		
		#self.linkMainViewToHead = 0
		self.mainViewLinkedToHead 	= 0
		
		self.phaseSpaceFilePath = phaseSpaceFilePath #'../Resources/'
		self.origin 	= [0,0,0];
		self.scale 		= [0.001,0.001,0.001];
		self.serverAddress = phaseSpaceIP; #'192.168.1.230';
		
		self.rigidFileNames_ridx = rigidFileNames_ridx  #['hmd-oculus.rb','paddle-hand.rb']
		self.rigidBodyShapes_ridx = rigidBodyShapes_ridx# ['sphere','cylinder']
		self.rigidBodySizes_ridx = rigidBodySizes_ridx #[[.1],[.03,.09]]
		self.rigidBodyToggleVisibility_ridx = rigidBodyToggleVisibility_ridx #[0,1]
		
		# Number of markers to look for
		self.owlParamMarkerCount = owlParamMarkerCount  #10
		self.owlParamFrequ = OWL_MAX_FREQUENCY
		self.owlParamInterp = 0
		self.owlParamMarkerCondThresh = 100
		
		# Init server
		#if (owlInit(self.serverAddress,0) < 0): 
		
		if (owlInit(self.serverAddress, OWL_MODE3) < 0): #Adds a lot of lag!
			print "Mocap: Could not connect to OWL Server"
			exit()
		
		if not owlGetStatus():
			print "Mocap: could not enable OWL_STREAMING, owlGetStatus returned: ", owlGetError()
			exit();
			
		####################################################################################################################
		####################################################################################################################
		# Set server parameters
		
		# set default frequency
		if( self.owlParamFrequ == 0 ):
			
			self.owlParamFrequ = OWL_MAX_FREQUENCY;
			
		#fi
		
		owlSetFloat(OWL_FREQUENCY, self.owlParamFrequ)
		# start streaming
		owlSetInteger(OWL_STREAMING, OWL_ENABLE)
		owlSetInteger(OWL_INTERPOLATION, self.owlParamInterp)
		
		#owlSetInteger(OWL_POSTPROCESS, OWL_ENABLE) Doesn't work.
		# some params: OWL_POSTPROCESS,  OWL_SLAVE) < 0):
		
		######################################################################
		######################################################################
		## Create rigid trackers
		
		for rigidIdx in range(len(self.rigidFileNames_ridx)):
			
			if( self.rigidFileNames_ridx[rigidIdx].find('hmd')>-1):
				# If it is the HMD rigid, create a rigidHMD object
				self.allRigids_ridx.append(rigidHMD(rigidIdx,
													self.phaseSpaceFilePath,
													self.rigidFileNames_ridx[rigidIdx],
													self.rigidBodyShapes_ridx[rigidIdx],
													self.rigidBodySizes_ridx[rigidIdx],
													self.rigidBodyToggleVisibility_ridx[rigidIdx]))
				self.rigidHMDIdx = rigidIdx
				print 'HMD RIGID CREATED'
			else:
				# A normal rigid body.  Create a rigidbody object
				self.allRigids_ridx.append(rigidBody(rigidIdx,
													self.phaseSpaceFilePath,
													self.rigidFileNames_ridx[rigidIdx],
													self.rigidBodyShapes_ridx[rigidIdx],
													self.rigidBodySizes_ridx[rigidIdx],
													self.rigidBodyToggleVisibility_ridx[rigidIdx]))
				
		### Track markers not on rigid bodies 
		# Fill allRigids_ridx with server data
		# Count up markers on rigid bodies, while building a list
		
		numRigidMarkers = 0;
		markersUsedInRigid = []
		#tempRigids = owlGetRigids();
		
		for rIdx in range(len(self.allRigids_ridx)):
			markersUsedInRigid.extend( self.allRigids_ridx[rIdx].markerID_midx );
		
		#print markersUsedInRigid;
		if( markersUsedInRigid > self.owlParamMarkerCount ): 'Mocap: More markers used by rigid bodies than in owlParamMarkerCount.'
		
		######################################################################
		######################################################################
		
		 ##Create a tracker object for individual markers
		 ##Note: we must continue indexing trackers at num rigids+1
		 ##Note: we must not track markers already on rigid bodies
		
		#trackerIdx = len(markersUsedInRigid)
		trackerIdx = len(self.allRigids_ridx)
		
		owlTrackeri(trackerIdx, OWL_CREATE, OWL_POINT_TRACKER)
		
		markerCount = 0;
		
		for i in xrange(self.owlParamMarkerCount):
			# if marker not already used in rigid...
			if markersUsedInRigid.count(i) == 0: 
				owlMarkeri(MARKER(trackerIdx,markerCount), OWL_SET_LED, i)
				print 'Mocap: marker: ' + str(i) + ' is unattached, and was added to a tracker.'
				markerCount += 1;
			else:
				pass
				#print 'Mocap: marker: ' + str(i) + ' is attached to a rigid body'
		
		owlTracker(trackerIdx, OWL_ENABLE);
		
		######################################################################
		######################################################################
		
		if not owlGetStatus():
			print "HelmetHandPhaseSpace, could not enable OWL_STREAMING, owlGetStatus returned: ", owlGetError()
			exit();

		# Setup a timer to update owl server
		self.callback(viz.TIMER_EVENT, self.refreshMarkerPositions)
		self.starttimer(0,0,viz.FOREVER)
		self.turnOn()
	
		### Link mainview to head?
		
		if( self.rigidHMDIdx > -1 ):
			self.linkMainViewToHead()
			#self.vizNodeLink = viz.link(self.allRigids_ridx[self.rigidHMDIdx].vizNode,viz.MainView)
			#self.mainViewLinkedToHead = 1
			
	# end init
	######################################################################
	######################################################################
	
	def getMarkerPosition(self,trackerID,markerID):
		
		globalMarkerID = rigidXMarkerID( trackerID, markerID );
		
		for mIdx in range(len(self.allMarkers_midx)):
			if( self.allMarkers_midx[mIdx].id == globalMarkerID ):
				
				return [self.allMarkers_midx[mIdx].x, self.allMarkers_midx[mIdx].y, self.allMarkers_midx[mIdx].z]
				
		print 'Marker not visible'
		
	def returnPointerToRigid(self,fileName):
		
		#  Accepts partial filenames, such as 'hmd' or 'paddle'
		#  Will return the first match found.
		
		for rigidIdx in range(len(self.rigidFileNames_ridx)):
			if( self.rigidFileNames_ridx[rigidIdx].find(fileName) > -1 ):
			#if( self.rigidFileNames_ridx[rigidIdx] == fileName):
				return self.allRigids_ridx[rigidIdx]
				
		print 'returnPointerToRigid: Could not find ' + fileName
		return 0
	
	def checkForRigid(self,fileName):
		return( self.returnPointerToRigid(fileName) )
		
	def __del__(self):
		self.quit()
			
	def turnOff(self):
		self.off = 1
		self.setEnabled(False)
		
		if self.mainViewLinkedToHead == 1:
			
			self.mainViewLinkedToHead = 0;
			self.allRigids_ridx[self.rigidHMDIdx].vizNodeLink.disable()

	def turnOn(self):
		self.off = 0
		self.setEnabled(True)

	def isOn(self):
		return not self.off

	def quit(self):
		print "Disconnecting"
		owlDone()

	def getOutput(self):
		
		return ( ' PhaseSpace: ' + `viz.MainView.getPosition()` + ' ' + `viz.MainView.getQuat()` );
	
	#fed

	
	
	def refreshMarkerPositions( self, num ):
		
		if (num == 0):
			
			# These are bools for whether to look for a rigid body
			
			numMarkersSeenThisRound = 1;
			markersSeen = [];
			#self.allMarkers_midx = []
			
			# There is a buffer of rigidsSeen and markersSeen
			# Empty this buffer and use latest information.
			while numMarkersSeenThisRound:
			
				markersSeen = owlGetMarkers();
				numMarkersSeenThisRound = len( markersSeen );
				numMarkersSeenLastRound = len( self.allMarkers_midx );
				
				
				if( numMarkersSeenThisRound > 0 ):
					
					tempMarkerVector = [];
					
					if( numMarkersSeenLastRound == 0 ):
						
						# The first time around, populate self.allMarkers
						self.allMarkers_midx = markersSeen;
						
						print ('mocap.refreshMarkerPositions: Getting marker data for first time.');
					
					else:
						
						for idx in range( 0, numMarkersSeenThisRound, 1 ):
							
							#self.allMarkers_midx = markersSeen;
							
							currentMarkersCondition = markersSeen[idx].cond;
							
							# Run a quality check!
							if( currentMarkersCondition > 0 ):
								
								# If the marker was seen
								# The highter the .cond, the poorer the track quality
								if( currentMarkersCondition < self.owlParamMarkerCondThresh ):
									
									self.allMarkers_midx = markersSeen; # Update the marker position
								
							#fi
									
						#rof
						
					#fi
					
					
					if( len(tempMarkerVector) ):
						
						self.allMarkers_midx = tempMarkerVector; # Testing this.  Not usre if it will work.
					
					#fi
			
				#fi
				
			#elihw
			
			
			rigidsSeen = [];
			numRigidSeen = 1;
			
			
			while numRigidSeen:
				
				rigidsSeen = owlGetRigids()			
				numRigidSeen = len(rigidsSeen);
				
				if( numRigidSeen ):
					
					#rigidTempContainer = rigidsSeen;
					for rIdx in range(0,len(rigidsSeen)):
						
						if( rigidsSeen[rIdx].cond > 0 and 
							rigidsSeen[rIdx].cond < self.owlParamMarkerCondThresh ):
							
							self.updateRigidBody(self.allRigids_ridx[rIdx],rigidsSeen[rIdx])
							
						#print str( rigidsSeen[rIdx].cond )
						#if( rigidsSeen[rIdx].cond < self.owlParamMarkerCondThresh ):
						#	self.updateRigidBody(self.allRigids_ridx[rIdx],rigidsSeen[rIdx])
						#else: print 'Mocap.refreshMarkerPositions(): Rigid body cond below thresh'
							
#			if self.isOn() and rigidTempContainer:
#				self.updateRigidBody(self.allRigids_ridx[0])
				
		else:
			print "HelmetHandPhaseSpace.__refreshMarkerPositions__(), YIKES, Unknown num(",num,")"

		
	def updateRigidBody(self,rigidBodyPointer,serverData):
		
		# Update server data.  
		rigidBodyPointer.serverData = serverData;
		
		pos = [  serverData.pose[0]*self.scale[0] + self.origin[0],
			     serverData.pose[1]*self.scale[1] + self.origin[1],
			    -serverData.pose[2]*self.scale[2] + self.origin[2] ];
		
		rot = [ serverData.pose[4], serverData.pose[5], -serverData.pose[6], -serverData.pose[3] ];
		
		# This updates position.
		rigidBodyPointer.updateVizNode( pos, rot );
	
	
	def resetRigid( self, fileName ):
		
		rigidBody = self.returnPointerToRigid( fileName );
		
		if( rigidBody ):
			
			rigidBody.resetRigid( self.allMarkers_midx );
		
		else:
			
			print ('Error: Rigid body not initialized');
			
		#fi
		
	#fed
		
	def saveRigid(self,fileName):
		rigidBody = self.returnPointerToRigid(fileName)
		
		if(rigidBody):
			rigidBody.saveNewDefaults()
		else: print 'Error: Rigid body not initialized'
	
	def rotateRigid(self,fileName,rotateByDegs_XYZ):
		
		rigidBody = self.returnPointerToRigid(fileName)
		
		if(rigidBody):
			rigidBody.rotateRigid(rotateByDegs_XYZ)
		else: print 'Error: Rigid body not initialized'
		
	
	def toggleRigidVisibility(self,fileName):
		
		rigidBody = self.returnPointerToRigid(fileName)
		
		if(rigidBody):
			rigidBody.toggleVisibility()
		else: print 'Rigid body not initialized'
		
	
	def toggleCalibrationCylinders(self):
		
		hmdRigid = self.returnPointerToRigid('hmd')
		
		if( hmdRigid ):
			print 'Toggling eye probes'
			hmdRigid.toggleCalibrationCylinders()
		else:
			print 'No HMD rigid body'
	
	def linkMainViewToHead(self):
		print 'Helmet-mainview linked'
		
		helmetRigid = self.allRigids_ridx[self.rigidHMDIdx]
		helmetRigid.vizNodeLink = viz.link(helmetRigid.vizNode,viz.MainView)
		
		helmetRigid.isVisible = 0
		helmetRigid.vizNode.visible( viz.OFF ) #Make the object visible.
		
		self.mainViewLinkedToHead = 1
		
	
	def detatchMainViewFromHead(self):
		print 'Helmet-mainview link removed'
		
		helmetRigid = self.allRigids_ridx[self.rigidHMDIdx]
		helmetRigid.vizNodeLink.remove()
		self.mainViewLinkedToHead = 0
		
		helmetRigid.vizNode.isVisible = 1
		helmetRigid.vizNode.visible( viz.ON ) #Make the object visible.
		
		self.headAxes = vizshape.addAxes(parent = self.allRigids_ridx[self.rigidHMDIdx].vizNode, pos=[0,0,0],scale=[0.2,0.2,0.2])
		rigidBody.toggleVisibility(self.allRigids_ridx[self.rigidHMDIdx])


if __name__ == "__main__":
	
	import vizact

	phaseSpaceIP = '192.168.1.230'
	owlParamMarkerCount = 20
	
	# Dir where textures and rb files are located
	phaseSpaceFilePath = 'Resources/'
	
	# Rb files in phaseSpaceFilePath
	#rigidFileNames_ridx = ['hmd-oculus.rb','paddle-hand.rb']
	rigidFileNames_ridx = ['hmd-nvis.rb','torch.rb']
	
	# Shapes defined in rigidbody._createVizNode()
	# Look there to see what shapes are accepted, or add more shapes
	#rigidBodyShapes_ridx = ['sphere','cylinder']
	rigidBodyShapes_ridx = ['sphere','cylinder']
	
	# Sizes must correspond to parameters for the vizshape in rigidBodyShapes_ridx 
	rigidBodySizes_ridx = [[.1],[.15,.03]]
	#rigidBodySizes_ridx = [.1]
	
	# Is the rigid body visible on startup?
	#rigidBodyToggleVisibility_ridx = [0,1]
	rigidBodyToggleVisibility_ridx = [0,1]
	
	# Start up the motion capture server
	mocap = mocapInterface(phaseSpaceIP,phaseSpaceFilePath,owlParamMarkerCount,
	rigidFileNames_ridx,rigidBodyShapes_ridx,rigidBodySizes_ridx,
	rigidBodyToggleVisibility_ridx)
	
	#torchRigid = mocap.returnPointerToRigid('torch')
	#viz.link(torchRigid.visNode,lightSource)
	
	# Draw the room using the environment module
	#import environment
	#environment = environment.room() 
	environment = viz.addChild('gallery.osgb')
	
	# Some parameters for reseting rigid bodies
	vizact.onkeydown( 'h', mocap.resetRigid, 'hmd' );
	vizact.onkeydown( 'j', mocap.saveRigid, 'hmd' );
	vizact.onkeydown( 'o', mocap.resetRigid, 'torch' );
	vizact.onkeydown( 'p', mocap.resetRigid, 'torch' );
	
	# Allows one to link to the HMD rigid body , or use keyboard/mouse to navigate
	vizact.onkeydown('l',mocap.linkMainViewToHead)
	vizact.onkeydown('d',mocap.detatchMainViewFromHead)
	
	trackerId = 0
	markerIDOnTracker = 0
	vizact.onkeydown('m',mocap.getMarkerPosition,trackerId ,markerIDOnTracker )# not well debugged yet!
	
	if( mocap.mainViewLinkedToHead ):
		'LInked'
		
		#import oculus
		#hmd = oculus.Rift()
		
		import nvis
		hmd = nvis.nvisorSX111()
		
		#print "HMD IPD: " + str(hmd.getIPD())
		#print "Mainvtiew IPD: " + str(viz.MainWindow.getIPD())

	viz.window.setFullscreenMonitor([1,2]) 
	viz.setMultiSample(4)
	
	viz.go(viz.FULLSCREEN)
	#Add a world axis with X,Y,Z labels
	world_axes = vizshape.addAxes()
	X = viz.addText3D('X',pos=[1.1,0,0],color=viz.RED,scale=[0.3,0.3,0.3],parent=world_axes)
	Y = viz.addText3D('Y',pos=[0,1.1,0],color=viz.GREEN,scale=[0.3,0.3,0.3],align=viz.ALIGN_CENTER_BASE,parent=world_axes)
	Z = viz.addText3D('Z',pos=[0,0,1.1],color=viz.BLUE,scale=[0.3,0.3,0.3],align=viz.ALIGN_CENTER_BASE,parent=world_axes)
import numpy as np

class transformation:
   
   def __init__(self):
      self.T = None
      self.transformationMatrix()
      
   def transformationMatrix(self):   
      #Some points in the robot frame
      Rx = np.array([0.386, 0.265, 0.681, 0.560, 0.601, 0.488])
      Ry = np.array([0.485, 0.491, 0.083, 0.085, -0.095, -0.138])
      Rz = np.array([0.132, 0.138, 0.006, 0.010, 0.109, 0.114])
      
      #Some points in the camera frame
      Cx = np.array([-0.390, -0.522, -0.076, -0.205, -0.144, -0.259])
      Cy = np.array([0.210, 0.203, 0.284, 0.281, 0.155, 0.141])
      Cz = np.array([1.277, 1.272, 0.880, 0.877, 0.723, 0.669])
      
      #A matrix for the camera frame, since we want to transform from camera frame to robot frame
      A = np.matrix([[np.sum(np.square(Cx)), np.sum(np.multiply(Cx,Cy)), np.sum(np.multiply(Cx,Cz)), np.sum(Cx)],
        [np.sum(np.multiply(Cx,Cy)), np.sum(np.square(Cy)), np.sum(np.multiply(Cy,Cz)), np.sum(Cy)],
        [np.sum(np.multiply(Cx,Cz)), np.sum(np.multiply(Cy,Cz)), np.sum(np.square(Cz)), np.sum(Cz)],
        [np.sum(Cx), np.sum(Cy), np.sum(Cz), 6]])
   
      #Intermediate vectors for the final transfomration matrix betweem the frames   
      V1 = np.array([np.sum(np.multiply(Rx,Cx)), np.sum(np.multiply(Rx,Cy)), np.sum(np.multiply(Rx,Cz)), np.sum(Rx)])
      V2 = np.array([np.sum(np.multiply(Ry,Cx)), np.sum(np.multiply(Ry,Cy)), np.sum(np.multiply(Ry,Cz)), np.sum(Ry)])
      V3 = np.array([np.sum(np.multiply(Rz,Cx)), np.sum(np.multiply(Rz,Cy)), np.sum(np.multiply(Rz,Cz)), np.sum(Rz)])
           
      rxx = np.dot(np.linalg.inv(A),np.reshape(V1, (4,1)))
      ryy = np.dot(np.linalg.inv(A),np.reshape(V2, (4,1)))
      rzz = np.dot(np.linalg.inv(A),np.reshape(V3, (4,1)))
      
      #The transformation matrix
      self.T = np.matrix([[rxx.item(0), rxx.item(1), rxx.item(2), rxx.item(3)],
                     [ryy.item(0), ryy.item(1), ryy.item(2), ryy.item(3)],
                     [rzz.item(0), rzz.item(1), rzz.item(2), rzz.item(3)],
                     [0, 0, 0, 1]])
      
      #print(np.dot(T,np.reshape(np.array([Cx.item(0), Cy.item(0), Cz.item(0), 1]), (4,1))))
           
   def inRobotFrame(self, x, y, z):
      #The function receives coordinates in the camera frame and results in coordinates in the robot frame
      transformationMatrix()
      res = np.dot(self.T,np.reshape(np.array([x, y, z, 1]), (4,1)))
      return res

frame = transformation()
print(frame.T)
res = frame.inRobotFrame(-0.390, 0.21, 1.277)   
print(res)
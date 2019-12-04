#!/usr/bin/env python 

#libs:
import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

class camera:
	def __init__(self):
		# create a node
		rospy.init_node('nodecv2', anonymous=True)
		# publisher object
		self.pub = rospy.Publisher('topiccv2', Image, queue_size=10)
		
		# bridge object
		self.bridge = CvBridge()

		# camera setup
		self.cap = cv2.VideoCapture(0)



	# pub metod
	def pub_img(self):
		cv2.namedWindow('window_1') #para abrir as janelas
		cv2.namedWindow('window_R')
		cv2.namedWindow('window_G')
		cv2.namedWindow('window_B')
		cv2.namedWindow('window_reconstruction')



		while True:
			ret, cv2_frame = self.cap.read()

			r = cv2_frame[:,:,2] # todas as linhas, todas as colunas do canal R (BGR - 0, 1 e 2)
			g = cv2_frame[:,:,1]
			b = cv2_frame[:,:,0]

			#r[50:100,:] = 255  # para fazer uma faixa preta de lado a lado da janela

			# Coloca em preto e branco, binarizacao
			#r[r>50] = 255	# 50 e o threshold limiar
			#r[r<255] = 0

			#b[b>50] = 255	# 50 e o threshold limiar
			#b[b<255] = 0

			#b[(b>100) & (g<200)] = 255
			#b[b<255] = 0

			x = cv2_frame	#atribui o tamanho da imagem original

			# Para reconstruir a imagem
			x[:,:,2] = r
			x[:,:,1] = g
			x[:,:,0] = b

			r = cv2.medianBlur(r,5)
						
			circles = cv2.HoughCircles(r,cv2.HOUGH_GRADIENT,1,3000, param1=30,param2=50,minRadius=20,maxRadius=150)
			circles = np.uint16(np.around(circles))
		
			for i in circles[0,:]:
				# draw the outer circle
				cv2.circle(cv2_frame,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(cv2_frame,(i[0],i[1]),2,(0,0,255),3)

			#cv2.circle(cv2_frame,(20, 20), 5, (0,255,0), -1)
			
			edges = cv2.Canny(r,50,150)	# detector de bordas
			cv2.imshow("window_R", r)	#imagem que identifica o vermelho
			cv2.imshow("window_G", g)	#imagem que identifica o verde
			cv2.imshow("window_B", b)	#imagem que identifica o azul
			cv2.imshow("window_reconstruction", x)	#reconstrucao da imagem
			cv2.imshow("windows_edges", edges)	#


			if cv2.waitKey(30) == ord('q'):
				break
			ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
			self.pub.publish(ros_frame)

# main function
if __name__ == '__main__':
    try:
		campub = camera()
		campub.pub_img()
    except rospy.ROSInterruptException:
      	pass	
	#print("Shutting down")
	cv2.destroyAllWindows()		
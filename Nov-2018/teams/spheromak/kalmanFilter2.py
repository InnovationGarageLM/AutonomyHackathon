def kalmanFilter(X_k1,P_k1,Z_k,R_k,deltaT,comHead,comVel):
	#Inputs
	#State Vector X_k1=([x], [xdot], [y], [ydot], [thetaoff]) units should be in cm and degrees
	#State Covariance P_k1=([n x n]) matrix of covariance
	#State Measurement Z_k=([x], [xdot], [y], [ydot],[thetaoff])
	#Measurement Covariance R_k=(n x n) matrix of covariance of measurement on the diag
	#Timestep
	#Commanded Heading comHead=(num) should be in degrees
	#Commanded Velocity comVel=(num) should be in cm
	#
	#Outputs
	#State Vector Xk_k
	#State Covariance Pk_k
	
	#importing Math Functions!!!!
	import numpy as np
	
	# Defining Parameters
	velScale=1
	modelQ=0.3
	# Computing Parameters
	#velRatio=X_k1(1)/X_k1(3)
	matrixQ=np.array([[modelQ,0,0,0],[0,modelQ,0,0],[0,0,modelQ,0],[0,0,0,modelQ]])
	
	# Computing Model Forcing Function for Velocity
	#xDotCom=velScale*comVel*np.sin(comHead/180*np.pi)
	#yDotCom=velScale*comVel*np.cos(comHead/180*np.pi)
	xDotCom=velScale*comVel*np.sin(comHead)
	yDotCom=velScale*comVel*np.cos(comHead)
		

	Uk=np.array([[0], [xDotCom], [0], [yDotCom]])
	Bk=np.array([[0,0,0,0],[0,1,0,0],[0,0,0,0],[0,0,0,1]])
	
	# Computing Measurement to State Matrix
	Hk=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
	
	# Computing State Transition Matrix
	Fk= np.array([[1,deltaT,0,0], [0,0,0,0],[0,0,1,deltaT],[0,0,0,0]])
	
	# Prediction Phase of the Kalman Filter
	Xk_k1=np.matmul(Fk,X_k1)+np.matmul(Bk,Uk)
	#print(Fk)
        #print(P_k1)
        #print(matrixQ)
        Pk_k1=np.matmul(Fk,np.matmul(P_k1,np.transpose(Fk)))+matrixQ
	
	# Update Phase
	# Computing the Residual Error and SumTotal Covariance Matrix
	Yk=Z_k-np.matmul(Hk,Xk_k1)
	Sk=R_k+np.matmul(Hk,np.matmul(Pk_k1,np.transpose(Hk)))
	
	# Kalman Gain Calculation
	#print(Pk_k1)
        #print(np.transpose(Hk))
        #print(np.linalg.inv(Sk))
        Kk=np.matmul(Pk_k1, np.matmul(np.transpose(Hk), np.linalg.inv(Sk)))
	#print(Kk)
        #print(Yk)
	# State Update
	Xk_k=Xk_k1+np.matmul(Kk,Yk)
	Pk_k=np.matmul(np.identity(4) - np.matmul(Kk, Hk), Pk_k1)
	

        #Xk_k = Z_k
	return Xk_k, Pk_k
	
def kalmanInertia(X_k1,P_k1,Z_k,R_k,deltaT,comHead,comVel):
	#Inputs
	#State Vector X_k1=([x], [xdot], [y], [ydot], [thetaoff]) units should be in cm and degrees
	#State Covariance P_k1=([n x n]) matrix of covariance
	#State Measurement Z_k=([x], [xdot], [y], [ydot],[thetaoff])
	#Measurement Covariance R_k=(n x n) matrix of covariance of measurement on the diag
	#Timestep
	#Commanded Heading comHead=(num) should be in degrees
	#Commanded Velocity comVel=(num) should be in cm
	#
	#Outputs
	#State Vector Xk_k
	#State Covariance Pk_k
	
	#importing Math Functions!!!!
	import numpy as np
	
	# Defining Parameters
	inertia = 0.5
	velScale=1
	modelQ=0
	# Computing Parameters
	#velRatio=X_k1(1)/X_k1(3)
	matrixQ=np.array([[modelQ,0,0,0],[0,modelQ,0,0],[0,0,modelQ,0],[0,0,0,modelQ]])
	
	# Computing Model Forcing Function for Velocity
	#xDotCom=velScale*comVel*np.sin(comHead/180*np.pi)
	#yDotCom=velScale*comVel*np.cos(comHead/180*np.pi)
	xDotCom=velScale*comVel*np.sin(comHead)
	yDotCom=velScale*comVel*np.cos(comHead)
		

	Uk=np.array([[0], [xDotCom], [0], [yDotCom]])
	Bk=np.array([[0,0,0,0],[0,1,0,0],[0,0,0,0],[0,0,0,1]]) * inertia
	
	# Computing Measurement to State Matrix
	Hk=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
	
	# Computing State Transition Matrix
	Fk= np.array([[1,deltaT,0,0], [0,(1- inertia),0,0],[0,0,1,deltaT],[0,0,0,(1-inertia)]])
	
	# Prediction Phase of the Kalman Filter
	Xk_k1=np.matmul(Fk,X_k1)+np.matmul(Bk,Uk)
	Pk_k1=np.matmul(Fk,np.matmul(P_k1,np.transpose(Fk)))+matrixQ
	
	# Update Phase
	# Computing the Residual Error and SumTotal Covariance Matrix
	Yk=Z_k-np.matmul(Hk,Xk_k1)
	Sk=R_k+np.matmul(Hk,np.matmul(Pk_k1,np.transpose(Hk)))
	
	# Kalman Gain Calculation
	Kk=np.matmul(Pk_k1, np.matmul(np.transpose(Hk), np.linalg.inv(Sk)))
	
	# State Update
	Xk_k=Xk_k1+np.matmul(Kk,Yk)
	Pk_k=np.matmul(np.identity(4) - np.matmul(Kk, Hk), Pk_k1)
	
	return Xk_k, Pk_k
	

	

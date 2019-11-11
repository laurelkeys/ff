#%% Vision-based formation control of quadrotors
# 
# Notes:
# See the tutorial for explanations.
# The simulation environment in UE4 must be running before executing this code.
#
# (C) Kaveh Fathian, 2017-2018.
# Email: kavehfathian@gmail.com
# Website: https://sites.google.com/view/kavehfathian



#%%

import Include.multirotor.setup_path 
import airsim

import os
import time
import numpy as np
import pprint
import time

import matlab.engine 
meng = matlab.engine.start_matlab() # Connect to matlab API



# %% Desired formation parameters

# Total number of UAVs
numUAV = 3

# Desired formation (equilateral triangle)
qs = matlab.double([[0, 4, 2],\
                    [0, 0, 3]])

# Adjacency matrix
Adjm = matlab.double([[0, 1, 1],\
                      [1, 0, 1],\
                      [1, 1, 0]])

# Initial positions of the quads
# NOTE: THIS MUST MATCH WITH THE SETTING FILE!
pos0 = np.zeros((numUAV,3))
for i in range(numUAV):
    pos0[i,0] = 588.0
    pos0[i,1] = -28.0 + 4.0*i
    pos0[i,2] = -125.0



#%% Find formaiton control gains

# Formation control gains
Am = meng.FindGains_Ver1_4(qs, Adjm, nargout=1)
print("Am:", Am)

# Conver Am to numpy array
A = np.asarray(Am)
Adj = np.asarray(Adjm)
print("Gain matrix calculated.")



#%% Connect to the AirSim simulator

client = airsim.MultirotorClient()
client.confirmConnection()

for i in range(numUAV):
    name = "UAV" + str(i+1)
    client.enableApiControl(True, name)
    client.armDisarm(True, name)
print("All UAVs have been initialized.")


# Hover
time.sleep(2) 

tout = 3 # Timeout in seconds
spd = 2 # Speed 

print("taking off...")
for i in range(numUAV):    
    name = "UAV" + str(i+1)
    print('Hovering', name)
    client.hoverAsync(vehicle_name = name)
    client.moveToPositionAsync(0, 0, -1, spd, timeout_sec = tout, vehicle_name = name)
print("All UAVs are hovering.")



# %% Increase altitude

tout = 10.0     # Timeout in seconds
spd = 4.0       # Speed
alt = -20.0     # Altitude

time.sleep(0.5) 

for i in range(numUAV):
    name = "UAV" + str(i+1)
    print('Moving', name)
    client.moveToPositionAsync(0, 0, alt, spd, timeout_sec = tout, vehicle_name = name)
print("UAVs reached desired altitude")



#%% Formation control loop

dcoll = 1.5     # Collision avaoidance activation distance 
rcoll = 0.7     # Collision avaoidance circle radius
gain = 1.0/3    # Control gain
alt = -20.0     # UAV altitude
duration = 0.5  # Max duration for applying input
vmax = 0.1      # Saturation velocity
save = 1        # Set to 1 to save onboard images, otherwise set to 0


# Initial Pause time
time.sleep(0.5) 

# Get Image data (Initialization)
for i in range(numUAV):
    name = "UAV" + str(i+1)
    # Scene vision image in uncompressed RGBA array
    img = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, 
                False, compress = False)], vehicle_name = name)[0] # airsim.types.ImageResponse
    print('Retrieved images of', name)

    img1d = np.fromstring(img.image_data_uint8, dtype=np.uint8)
    img_rgb = np.flipud(img1d.reshape(img.height, img.width, -1))
    print(f"img1d: {img1d.shape}, img_rgb: {img_rgb.shape}, img...uint8: {len(img.image_data_uint8)}")
    airsim.write_png(f'out0-{i}.png', img_rgb)
    airsim.write_file(f'out1-{i}.png', img.image_data_uint8)
    
    # (flattened) array of image bytes
    if i == 0:
        imgArray = img.image_data_uint8
    else:
        imgArray = imgArray + img.image_data_uint8
    print(f"imgArray: {len(imgArray)}")
    
    # Scene vision image in compressed RGBA array
    img_compr = client.simGetImage("0", airsim.ImageType.Scene, vehicle_name = name) # bytes
    airsim.write_file(f'out2-{i}.png', airsim.string_to_uint8_array(img_compr))

imgWidth = img.width
imgHeight = img.height


# Formation control
itr = 0
while True: 
    
    itr = itr + 1
    print("itr = ", itr)
    if itr == 1:
        print("Starting a parallel pool...")
    
    # Get UAV positions for collision avoidance
    q = np.zeros(3*numUAV) # Preallocate state vectors
    qo = np.zeros(4*numUAV) # Preallocate orientation vectors
    qxy = np.zeros(2*numUAV)
    for i in range(numUAV):
        name = "UAV" + str(i+1)
        
        # Get x-y-z coordinates
        pos = client.simGetGroundTruthKinematics(vehicle_name = name)   
        qi = np.array([pos['position']['x_val'], pos['position']['y_val'], pos['position']['z_val']])
        qoi = np.array([pos['orientation']['w_val'], pos['orientation']['x_val'], pos['orientation']['y_val'], pos['orientation']['z_val']])
        
        # Add initial coordinates
        qd = qi + pos0[i,:]
        
        # 3D and 2D state vector
        q[3*i:3*i+3] =  qd.copy()
        qxy[2*i:2*i+2] = np.array([qd[0], qd[1]])  
        qo[4*i:4*i+4] =  qoi.copy()
    
    
    # Estimate relative positions using onboard images
    for i in range(numUAV):
        name = "UAV" + str(i+1)
        # Scene vision image in uncompressed RGBA array
        img = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, 
                False, False)], vehicle_name = name)[0] # airsim.types.ImageResponse
        print('Retrieved images of', name)
        
        img1d = np.fromstring(img.image_data_uint8, dtype=np.uint8)
        img_rgb = np.flipud(img1d.reshape(img.height, img.width, -1))
        print(f"img1d: {img1d.shape}, img_rgb: {img_rgb.shape}, img...uint8: {len(img.image_data_uint8)}")
        airsim.write_png(f'out3-{i}.png', img_rgb)
        airsim.write_file(f'out4-{i}.png', img.image_data_uint8)
        
        # (flattened) array of image bytes
        if i == 0:
            imgArray = img.image_data_uint8
        else:
            imgArray = imgArray + img.image_data_uint8
        print(f"imgArray: {len(imgArray)}")

    # ref.: https://www.mathworks.com/help/matlab/matlab_external/pass-data-to-matlab-from-python.html
    try:
        # meng.edit('GetRelativePose_Ver1_7')
        print("===========--=======--")
        npImgArray = np.frombuffer(imgArray, dtype='uint8')
        print(type(imgArray), len(imgArray), npImgArray.shape, npImgArray.dtype)
        # _imgArray = npImgArray.tobytes()
        # _imgArray = npImgArray.tolist()
        _imgArray = bytearray(npImgArray)
        print(type(_imgArray), len(_imgArray))
        print("imgWidth:", imgWidth)
        print("imgHeight:", imgHeight)
        print("save:", save)
        print("Adjm:", Adjm)
        print("itr:", itr)
        print("===========--=======--")
        Qm, Tm, flagm = meng.GetRelativePose_Ver1_7(_imgArray, imgWidth, imgHeight, save, Adjm, itr, nargout=3)
        T = np.asarray(Tm)
        flag = np.asarray(flagm).flatten()
        print('T:')
        print(T)
    except Exception as e:
        meng.quit()
        client.reset()
        print("Exception:", e)
        exit(0)
    
    
    # Transform recovered coordinates to world frame in order to apply the control.
    # AirSim uses NED (North-East-Down) frame, and the front camera has EDN 
    # (East-Down-Notrth) frame. So we need to swao columns of T.
    Tw = np.array([T[:,2], T[:,0], T[:,1]]).T
    
    
    # Calculate distributed control
    dqxy = np.zeros(2*numUAV) # Preallocate vectors
    for i in range(numUAV):
        if flag[i] == 1:
            # 3D and 2D state vector
            qi = Tw[i*numUAV: (i+1)*numUAV, :].flatten() 
            qxyi = Tw[i*numUAV: (i+1)*numUAV, 0:2].flatten() 
            
            # Control
            dqxyi = A[i*2: i*2+2,:].dot(qxyi)
            dqxy[2*i:2*i+2] = gain * dqxyi
    if save == 1:
        np.save("SavedData/u"+str(itr),dqxy) # Save control
    
    
    # Collision avoidance  # meng.edit('ColAvoid_Ver2_1')    
    um = meng.ColAvoid_Ver2_1(dqxy.tolist(),qxy.tolist(), dcoll, rcoll, nargout=1)
    u = np.asarray(um).flatten()     
#    u = dqxy
    
    
    # Saturate velociy control command
    for i in range(numUAV):
        # Find norm of control vector for each UAV
        ui = u[2*i : 2*i+2]
        vel = np.linalg.norm(ui)
        if vel > vmax:
            u[2*i : 2*i+2] = (vmax / vel) * u[2*i : 2*i+2]
    if save == 1:
        np.save("SavedData/um"+str(itr),u) # Save modified control
    
    
#     Apply control command    
    for i in range(numUAV):
        name = "UAV" + str(i+1)
        client.moveByVelocityZAsync(u[2*i], u[2*i+1], alt, duration, vehicle_name = name) # Motion at fixed altitude
    
    print()



#%% Terminate

meng.quit()
client.reset()

















































































    


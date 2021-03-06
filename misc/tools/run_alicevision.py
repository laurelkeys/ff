# NOTE this file was taken from http://filmicworlds.com/blog/command-line-photogrammetry-with-alicevision/

import os
import sys


def SilentMkdir(theDir):
	try:
		os.mkdir(theDir)
	except:
		pass
	return 0

def Run_00_CameraInit(baseDir,binDir,srcImageDir):
	SilentMkdir(baseDir + "/00_CameraInit")

	binName = binDir + "\\aliceVision_cameraInit.exe"

	dstDir = baseDir + "/00_CameraInit/"
	cmdLine = binName
	cmdLine = cmdLine + " --defaultFieldOfView 45.0 --verboseLevel info --sensorDatabase \"\" --allowSingleView 1"
	cmdLine = cmdLine + " --imageFolder \"" + srcImageDir + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "cameraInit.sfm\""
	print(cmdLine)
	os.system(cmdLine)

	return 0

def Run_01_FeatureExtraction(baseDir,binDir, numImages):
	SilentMkdir(baseDir + "/01_FeatureExtraction")

	srcSfm = baseDir + "/00_CameraInit/cameraInit.sfm"

	binName = binDir + "\\aliceVision_featureExtraction.exe"

	dstDir = baseDir + "/01_FeatureExtraction/"

	cmdLine = binName
	cmdLine = cmdLine + " --describerTypes sift --forceCpuExtraction True --verboseLevel info --describerPreset normal"
	cmdLine = cmdLine + " --rangeStart 0 --rangeSize " + str(numImages)
	cmdLine = cmdLine + " --input \"" + srcSfm + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "\""
	print(cmdLine)
	os.system(cmdLine)

	return 0

def Run_02_ImageMatching(baseDir,binDir):
	SilentMkdir(baseDir + "/02_ImageMatching")

	srcSfm = baseDir + "/00_CameraInit/cameraInit.sfm"
	srcFeatures = baseDir + "/01_FeatureExtraction/"
	dstMatches = baseDir + "/02_ImageMatching/imageMatches.txt"

	binName = binDir + "\\aliceVision_imageMatching.exe"

	cmdLine = binName
	cmdLine = cmdLine + " --minNbImages 200 --tree "" --maxDescriptors 500 --verboseLevel info --weights "" --nbMatches 50"
	cmdLine = cmdLine + " --input \"" + srcSfm + "\""
	cmdLine = cmdLine + " --featuresFolder \"" + srcFeatures + "\""
	cmdLine = cmdLine + " --output \"" + dstMatches + "\""

	print(cmdLine)
	os.system(cmdLine)

	return 0

def Run_03_FeatureMatching(baseDir,binDir):
	SilentMkdir(baseDir + "/03_FeatureMatching")

	srcSfm = baseDir + "/00_CameraInit/cameraInit.sfm"
	srcFeatures = baseDir + "/01_FeatureExtraction/"
	srcImageMatches = baseDir + "/02_ImageMatching/imageMatches.txt"
	dstMatches = baseDir + "/03_FeatureMatching"

	binName = binDir + "\\aliceVision_featureMatching.exe"

	cmdLine = binName
	cmdLine = cmdLine + " --verboseLevel info --describerTypes sift --maxMatches 0 --exportDebugFiles False --savePutativeMatches False --guidedMatching False"
	cmdLine = cmdLine + " --geometricEstimator acransac --geometricFilterType fundamental_matrix --maxIteration 2048 --distanceRatio 0.8"
	cmdLine = cmdLine + " --photometricMatchingMethod ANN_L2"
	cmdLine = cmdLine + " --imagePairsList \"" + srcImageMatches + "\""
	cmdLine = cmdLine + " --input \"" + srcSfm + "\""
	cmdLine = cmdLine + " --featuresFolders \"" + srcFeatures + "\""
	cmdLine = cmdLine + " --output \"" + dstMatches + "\""

	print(cmdLine)
	os.system(cmdLine)
	return 0

def Run_04_StructureFromMotion(baseDir,binDir):
	SilentMkdir(baseDir + "/04_StructureFromMotion")

	srcSfm = baseDir + "/00_CameraInit/cameraInit.sfm"
	srcFeatures = baseDir + "/01_FeatureExtraction/"
	srcImageMatches = baseDir + "/02_ImageMatching/imageMatches.txt"
	srcMatches = baseDir + "/03_FeatureMatching"
	dstDir = baseDir + "/04_StructureFromMotion"

	binName = binDir + "\\aliceVision_incrementalSfm.exe"

	cmdLine = binName
	cmdLine = cmdLine + " --minAngleForLandmark 2.0 --minNumberOfObservationsForTriangulation 2 --maxAngleInitialPair 40.0 --maxNumberOfMatches 0 --localizerEstimator acransac --describerTypes sift --lockScenePreviouslyReconstructed False --localBAGraphDistance 1"
	cmdLine = cmdLine + " --initialPairA "" --initialPairB "" --interFileExtension .ply --useLocalBA True"
	cmdLine = cmdLine + " --minInputTrackLength 2 --useOnlyMatchesFromInputFolder False --verboseLevel info --minAngleForTriangulation 3.0 --maxReprojectionError 4.0 --minAngleInitialPair 5.0"

	cmdLine = cmdLine + " --input \"" + srcSfm + "\""
	cmdLine = cmdLine + " --featuresFolders \"" + srcFeatures + "\""
	cmdLine = cmdLine + " --matchesFolders \"" + srcMatches + "\""
	cmdLine = cmdLine + " --outputViewsAndPoses \"" + dstDir + "/cameras.sfm\""
	cmdLine = cmdLine + " --extraInfoFolder \"" + dstDir + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "/bundle.sfm\""

	print(cmdLine)
	os.system(cmdLine)
	return 0

def Run_05_PrepareDenseScene(baseDir,binDir):
	SilentMkdir(baseDir + "/05_PrepareDenseScene")


	#srcSfm = baseDir + "/04_StructureFromMotion/cameras.sfm"
	srcSfm = baseDir + "/04_StructureFromMotion/bundle.sfm"
	dstDir = baseDir + "/05_PrepareDenseScene"

	binName = binDir + "\\aliceVision_prepareDenseScene.exe"

	cmdLine = binName
	cmdLine = cmdLine + " --verboseLevel info"
	cmdLine = cmdLine + " --input \"" + srcSfm + "\""
	cmdLine = cmdLine + " --output \"" + dstDir +"\""

	print(cmdLine)
	os.system(cmdLine)
	return 0

def Run_06_CameraConnection(baseDir,binDir):
	SilentMkdir(baseDir + "/06_CameraConnection")

	srcIni = baseDir + "/05_PrepareDenseScene/mvs.ini"

	# This step kindof breaks the directory structure. Tt creates
	# a camsPairsMatrixFromSeeds.bin file in in the same file as mvs.ini
	binName = binDir + "\\aliceVision_cameraConnection.exe"

	cmdLine = binName
	cmdLine = cmdLine + " --verboseLevel info"
	cmdLine = cmdLine + " --ini \"" + srcIni + "\""

	print(cmdLine)
	os.system(cmdLine)
	return 0

def Run_07_DepthMap(baseDir,binDir,numImages,groupSize):
	SilentMkdir(baseDir + "/07_DepthMap")

	numGroups = (numImages + (groupSize-1))/groupSize

	srcIni = baseDir + "/05_PrepareDenseScene/mvs.ini"
	binName = binDir + "\\aliceVision_depthMapEstimation.exe"
	dstDir = baseDir + "/07_DepthMap"

	cmdLine = binName
	cmdLine = cmdLine + " --sgmGammaC 5.5 --sgmWSH 4 --refineGammaP 8.0 --refineSigma 15 --refineNSamplesHalf 150 --sgmMaxTCams 10 --refineWSH 3 --downscale 2 --refineMaxTCams 6 --verboseLevel info --refineGammaC 15.5 --sgmGammaP 8.0"
	cmdLine = cmdLine + " --refineNiters 100 --refineNDepthsToRefine 31 --refineUseTcOrRcPixSize False"

	cmdLine = cmdLine + " --ini \"" + srcIni + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "\""


	for groupIter in range(numGroups):
		groupStart = groupSize * groupIter
		groupSize = min(groupSize,numImages - groupStart)
		print("DepthMap Group %d/%d: %d, %d" % (groupIter, numGroups, groupStart, groupSize))

		cmd = cmdLine + (" --rangeStart %d --rangeSize %d" % (groupStart,groupSize))
		print(cmd)
		os.system(cmd)


	#cmd = "aliceVision_depthMapEstimation  --sgmGammaC 5.5 --sgmWSH 4 --refineGammaP 8.0 --refineSigma 15 --refineNSamplesHalf 150 --sgmMaxTCams 10 --refineWSH 3 --downscale 2 --refineMaxTCams 6 --verboseLevel info --refineGammaC 15.5 --sgmGammaP 8.0 --ini \"c:/users/geforce/appdata/local/temp/MeshroomCache/PrepareDenseScene/4f0d6d9f9d072ed05337fd7c670811b1daa00e62/mvs.ini\" --refineNiters 100 --refineNDepthsToRefine 31 --refineUseTcOrRcPixSize False --output \"c:/users/geforce/appdata/local/temp/MeshroomCache/DepthMap/18f3bd0a90931bd749b5eda20c8bf9f6dab63af9\" --rangeStart 0 --rangeSize 3"
	#cmd = binName + " --sgmGammaC 5.5 --sgmWSH 4 --refineGammaP 8.0 --refineSigma 15 --refineNSamplesHalf 150 --sgmMaxTCams 10 --refineWSH 3 --downscale 2 --refineMaxTCams 6 --verboseLevel info --refineGammaC 15.5 --sgmGammaP 8.0 --ini \"c:/users/geforce/appdata/local/temp/MeshroomCache/PrepareDenseScene/4f0d6d9f9d072ed05337fd7c670811b1daa00e62/mvs.ini\" --refineNiters 100 --refineNDepthsToRefine 31 --refineUseTcOrRcPixSize False --output \"build_files/07_DepthMap/\" --rangeStart 0 --rangeSize 3"
	#cmd = binName + " --sgmGammaC 5.5 --sgmWSH 4 --refineGammaP 8.0 --refineSigma 15 --refineNSamplesHalf 150 --sgmMaxTCams 10 --refineWSH 3 --downscale 2 --refineMaxTCams 6 --verboseLevel info --refineGammaC 15.5 --sgmGammaP 8.0 --ini \"" + srcIni + "\" --refineNiters 100 --refineNDepthsToRefine 31 --refineUseTcOrRcPixSize False --output \"build_files/07_DepthMap/\" --rangeStart 0 --rangeSize 3"
	#print(cmd)
	#os.system(cmd)


	return 0

def Run_08_DepthMapFilter(baseDir,binDir):
	SilentMkdir(baseDir + "/08_DepthMapFilter")

	binName = binDir + "\\aliceVision_depthMapFiltering.exe"
	dstDir = baseDir + "/08_DepthMapFilter"
	srcIni = baseDir + "/05_PrepareDenseScene/mvs.ini"
	srcDepthDir = baseDir + "/07_DepthMap"

	cmdLine = binName
	cmdLine = cmdLine + " --minNumOfConsistensCamsWithLowSimilarity 4"
	cmdLine = cmdLine + " --minNumOfConsistensCams 3 --verboseLevel info --pixSizeBall 0"
	cmdLine = cmdLine + " --pixSizeBallWithLowSimilarity 0 --nNearestCams 10"

	cmdLine = cmdLine + " --ini \"" + srcIni + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "\""
	cmdLine = cmdLine + " --depthMapFolder \"" + srcDepthDir + "\""

	print(cmdLine)
	os.system(cmdLine)
	return 0

def Run_09_Meshing(baseDir,binDir):
	SilentMkdir(baseDir + "/09_Meshing")

	binName = binDir + "\\aliceVision_meshing.exe"
	srcIni = baseDir + "/05_PrepareDenseScene/mvs.ini"
	srcDepthFilterDir = baseDir + "/08_DepthMapFilter"
	srcDepthMapDir = baseDir + "/07_DepthMap"

	dstDir = baseDir + "/09_Meshing"

	cmdLine = binName
	cmdLine = cmdLine + " --simGaussianSizeInit 10.0 --maxInputPoints 50000000 --repartition multiResolution"
	cmdLine = cmdLine + " --simGaussianSize 10.0 --simFactor 15.0 --voteMarginFactor 4.0 --contributeMarginFactor 2.0 --minStep 2 --pixSizeMarginFinalCoef 4.0 --maxPoints 5000000 --maxPointsPerVoxel 1000000 --angleFactor 15.0 --partitioning singleBlock"
	cmdLine = cmdLine + " --minAngleThreshold 1.0 --pixSizeMarginInitCoef 2.0 --refineFuse True --verboseLevel info"

	cmdLine = cmdLine + " --ini \"" + srcIni + "\""
	cmdLine = cmdLine + " --depthMapFilterFolder \"" + srcDepthFilterDir + "\""
	cmdLine = cmdLine + " --depthMapFolder \"" + srcDepthMapDir + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "/mesh.obj\""

	print(cmdLine)
	os.system(cmdLine)
	return 0

def Run_10_MeshFiltering(baseDir,binDir):
	SilentMkdir(baseDir + "/10_MeshFiltering")

	binName = binDir + "\\aliceVision_meshFiltering.exe"

	srcMesh = baseDir + "/09_Meshing/mesh.obj"
	dstMesh = baseDir + "/10_MeshFiltering/mesh.obj"

	cmdLine = binName
	cmdLine = cmdLine + " --verboseLevel info --removeLargeTrianglesFactor 60.0 --iterations 5 --keepLargestMeshOnly True"
	cmdLine = cmdLine + " --lambda 1.0"

	cmdLine = cmdLine + " --input \"" + srcMesh + "\""
	cmdLine = cmdLine + " --output \"" + dstMesh + "\""

	print(cmdLine)
	os.system(cmdLine)

	return 0

def Run_11_Texturing(baseDir,binDir):
	SilentMkdir(baseDir + "/11_Texturing")

	binName = binDir + "\\aliceVision_texturing.exe"

	srcMesh = baseDir + "/10_MeshFiltering/mesh.obj"
	srcRecon = baseDir + "/09_Meshing/denseReconstruction.bin"
	srcIni = baseDir + "/05_PrepareDenseScene/mvs.ini"
	dstDir = baseDir + "/11_Texturing"

	cmdLine = binName
	cmdLine = cmdLine + " --textureSide 8192"
	cmdLine = cmdLine + " --downscale 2 --verboseLevel info --padding 15"
	cmdLine = cmdLine + " --unwrapMethod Basic --outputTextureFileType png --flipNormals False --fillHoles False"

	cmdLine = cmdLine + " --inputDenseReconstruction \"" + srcRecon + "\""
	cmdLine = cmdLine + " --inputMesh \"" + srcMesh + "\""
	cmdLine = cmdLine + " --ini \"" + srcIni + "\""
	cmdLine = cmdLine + " --output \"" + dstDir + "\""

	print(cmdLine)
	os.system(cmdLine)

	return 0

def main():
	print("Prepping Scan, v2.")

	print(sys.argv)

	print (len(sys.argv))
	if (len(sys.argv) != 6):
		print("usage: python run_alicevision.py <baseDir> <imgDir> <binDir> <numImages> <runStep>")
		print("Must pass 6 arguments.")
		sys.exit(0)
	baseDir = sys.argv[1]
	srcImageDir = sys.argv[2]
	binDir = sys.argv[3]
	numImages = int(sys.argv[4])
	runStep = sys.argv[5]

	print("Base dir  : %s" % baseDir)
	print("Image dir : %s" % srcImageDir)
	print("Bin dir   : %s" % binDir)
	print("Num images: %d" % numImages)
	print("Step      : %s" % runStep)




	SilentMkdir(baseDir)

	if runStep == "runall":
		Run_00_CameraInit(baseDir,binDir,srcImageDir)
		Run_01_FeatureExtraction(baseDir,binDir,numImages)
		Run_02_ImageMatching(baseDir,binDir)
		Run_03_FeatureMatching(baseDir,binDir)
		Run_04_StructureFromMotion(baseDir,binDir)
		Run_05_PrepareDenseScene(baseDir,binDir)

		Run_06_CameraConnection(baseDir,binDir)

		Run_07_DepthMap(baseDir,binDir,numImages,3)
		Run_08_DepthMapFilter(baseDir,binDir)
		Run_09_Meshing(baseDir,binDir)
		Run_10_MeshFiltering(baseDir,binDir)

		Run_11_Texturing(baseDir,binDir)
	elif runStep == "run00":
		Run_00_CameraInit(baseDir,binDir,srcImageDir)

	elif runStep == "run01":
		Run_01_FeatureExtraction(baseDir,binDir,numImages)
	elif runStep == "run02":
		Run_02_ImageMatching(baseDir,binDir)
	elif runStep == "run03":
		Run_03_FeatureMatching(baseDir,binDir)
	elif runStep == "run04":
		Run_04_StructureFromMotion(baseDir,binDir)
	elif runStep == "run05":
		Run_05_PrepareDenseScene(baseDir,binDir)

	elif runStep == "run06":
		Run_06_CameraConnection(baseDir,binDir)

	elif runStep == "run07":
		Run_07_DepthMap(baseDir,binDir,numImages,3)
	elif runStep == "run08":
		Run_08_DepthMapFilter(baseDir,binDir)
	elif runStep == "run09":
		Run_09_Meshing(baseDir,binDir)
	elif runStep == "run10":
		Run_10_MeshFiltering(baseDir,binDir)

	elif runStep == "run11":
		Run_11_Texturing(baseDir,binDir)

	else:
		print("Invalid Step: %s" % runStep)





	#print("running")
	#Run_00_CameraInit(baseDir,binDir,srcImageDir)
	#Run_01_FeatureExtraction(baseDir,binDir,numImages)
	#Run_02_ImageMatching(baseDir,binDir)
	#Run_03_FeatureMatching(baseDir,binDir)
	#Run_04_StructureFromMotion(baseDir,binDir)
	#Run_05_PrepareDenseScene(baseDir,binDir)

	#Run_06_CameraConnection(baseDir,binDir)

	#Run_07_DepthMap(baseDir,binDir,numImages,3)
	#Run_08_DepthMapFilter(baseDir,binDir)
	#Run_09_Meshing(baseDir,binDir)
	#Run_10_MeshFiltering(baseDir,binDir)

	#Run_11_Texturing(baseDir,binDir)
	return 0



main()



